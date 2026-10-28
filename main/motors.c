#include "motors.h"

#include "board_config.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "motors";

#if BOARD_PWM_BITS != 10
#error "motors.c assumes a 10-bit LEDC timer (duty 0..1024)"
#endif

#define MOTOR_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_TIMER LEDC_TIMER_0
#define LEFT_CHANNEL LEDC_CHANNEL_0
#define RIGHT_CHANNEL LEDC_CHANNEL_1

typedef struct {
    gpio_num_t in_a;
    gpio_num_t in_b;
    ledc_channel_t channel;
    int polarity;
    int last_sign;
} motor_side_t;

static motor_side_t s_left = {
    .in_a = BOARD_LEFT_IN1_GPIO,
    .in_b = BOARD_LEFT_IN2_GPIO,
    .channel = LEFT_CHANNEL,
    .polarity = BOARD_MOTOR_LEFT_POLARITY,
    .last_sign = 0,
};

static motor_side_t s_right = {
    .in_a = BOARD_RIGHT_IN1_GPIO,
    .in_b = BOARD_RIGHT_IN2_GPIO,
    .channel = RIGHT_CHANNEL,
    .polarity = BOARD_MOTOR_RIGHT_POLARITY,
    .last_sign = 0,
};

static bool s_ready;
static bool s_duty_warned;

static void set_enable(ledc_channel_t channel, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(MOTOR_MODE, channel, duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(MOTOR_MODE, channel);
    }
    if (err != ESP_OK && !s_duty_warned) {
        s_duty_warned = true;
        ESP_LOGW(TAG, "pwm update failed: %s", esp_err_to_name(err));
    }
}

static uint32_t map_start_duty(int magnitude)
{
    if (magnitude <= 0) {
        return 0;
    }
    if (magnitude > BOARD_DUTY_MAX) {
        magnitude = BOARD_DUTY_MAX;
    }
    int span = BOARD_DUTY_MAX - BOARD_MOTOR_START_DUTY;
    int scaled = BOARD_MOTOR_START_DUTY + (magnitude * span) / BOARD_DUTY_MAX;
    if (scaled > BOARD_DUTY_MAX) {
        scaled = BOARD_DUTY_MAX;
    }
    return (uint32_t)scaled;
}

static void brake_side(motor_side_t *side)
{
    gpio_set_level(side->in_a, 0);
    gpio_set_level(side->in_b, 0);
    set_enable(side->channel, BOARD_DUTY_MAX);
    side->last_sign = 0;
}

static void drive_side(motor_side_t *side, int signed_cmd)
{
    int sign = (signed_cmd > 0) - (signed_cmd < 0);
    if (sign == 0) {
        brake_side(side);
        return;
    }

    /* One control cycle of brake when the H-bridge reverses, so the L298N is not asked to swap rails instantly. */
    if (side->last_sign != 0 && sign != side->last_sign) {
        brake_side(side);
        return;
    }

    int magnitude = signed_cmd >= 0 ? signed_cmd : -signed_cmd;
    uint32_t duty = map_start_duty(magnitude);

    /* Drop enable before flipping direction so a brake (enable already high) cannot full-power the new rail. */
    if (sign != side->last_sign) {
        set_enable(side->channel, 0);
    }
    if (sign > 0) {
        gpio_set_level(side->in_a, 1);
        gpio_set_level(side->in_b, 0);
    } else {
        gpio_set_level(side->in_a, 0);
        gpio_set_level(side->in_b, 1);
    }
    set_enable(side->channel, duty);
    side->last_sign = sign;
}

static int clamp_command(int cmd)
{
    if (cmd > -BOARD_MOTOR_CMD_DEADBAND && cmd < BOARD_MOTOR_CMD_DEADBAND) {
        return 0;
    }
    if (cmd > BOARD_DUTY_MAX) {
        return BOARD_DUTY_MAX;
    }
    if (cmd < -BOARD_DUTY_MAX) {
        return -BOARD_DUTY_MAX;
    }
    return cmd;
}

esp_err_t motors_init(void)
{
    uint64_t mask = (1ULL << BOARD_LEFT_IN1_GPIO) | (1ULL << BOARD_LEFT_IN2_GPIO) |
                    (1ULL << BOARD_RIGHT_IN1_GPIO) | (1ULL << BOARD_RIGHT_IN2_GPIO);
    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "direction gpios");
    gpio_set_level(BOARD_LEFT_IN1_GPIO, 0);
    gpio_set_level(BOARD_LEFT_IN2_GPIO, 0);
    gpio_set_level(BOARD_RIGHT_IN1_GPIO, 0);
    gpio_set_level(BOARD_RIGHT_IN2_GPIO, 0);

    ledc_timer_config_t timer = {
        .speed_mode = MOTOR_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = MOTOR_TIMER,
        .freq_hz = BOARD_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "pwm timer");

    const gpio_num_t enable_pins[2] = {BOARD_LEFT_EN_GPIO, BOARD_RIGHT_EN_GPIO};
    const ledc_channel_t channels[2] = {LEFT_CHANNEL, RIGHT_CHANNEL};
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t channel = {
            .gpio_num = enable_pins[i],
            .speed_mode = MOTOR_MODE,
            .channel = channels[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = MOTOR_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "pwm channel");
    }

    s_ready = true;
    motors_brake();
    ESP_LOGI(TAG, "L298N pwm %d Hz, %d-bit, start duty %d",
             BOARD_PWM_FREQ_HZ, BOARD_PWM_BITS, BOARD_MOTOR_START_DUTY);
    return ESP_OK;
}

void motors_set_signed(int left, int right)
{
    if (!s_ready) {
        return;
    }
    drive_side(&s_left, clamp_command(left) * s_left.polarity);
    drive_side(&s_right, clamp_command(right) * s_right.polarity);
}

void motors_brake(void)
{
    if (!s_ready) {
        return;
    }
    brake_side(&s_left);
    brake_side(&s_right);
}
