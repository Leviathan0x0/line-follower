/*
 * ESP32-S3 line follower, one translation unit.
 *
 * This is the same firmware as main/. The default ESP-IDF build still
 * compiles the files in main/. Do not add this file to that component:
 * the public functions would be defined twice.
 *
 * Static names that were private to more than one of those files are
 * prefixed here with the file they came from (motors_s_ready, follower_TAG).
 * Everything else is unchanged.
 */

/* ---- board_config.h ---- */

#include "driver/gpio.h"

/*
 * ESP32-S3 DevKit line follower.
 *
 * Sensor 0 is the far-left channel when you face the direction of travel.
 * The array belongs on the front lip of the chassis, 8–12 mm above the track.
 *
 * Assumed motors are 6 V N20 gearmotors. The L298N drops about 2 V, so the
 * motor supply is a 2S pack (see README), not the 5 V logic rail.
 */

#define BOARD_SENSOR_COUNT 5

#define BOARD_SENSOR0_GPIO GPIO_NUM_1
#define BOARD_SENSOR1_GPIO GPIO_NUM_2
#define BOARD_SENSOR2_GPIO GPIO_NUM_4
#define BOARD_SENSOR3_GPIO GPIO_NUM_5
#define BOARD_SENSOR4_GPIO GPIO_NUM_6

#define BOARD_LEFT_IN1_GPIO GPIO_NUM_7
#define BOARD_LEFT_IN2_GPIO GPIO_NUM_8
#define BOARD_LEFT_EN_GPIO  GPIO_NUM_9

#define BOARD_RIGHT_IN1_GPIO GPIO_NUM_10
#define BOARD_RIGHT_IN2_GPIO GPIO_NUM_11
#define BOARD_RIGHT_EN_GPIO  GPIO_NUM_12

#define BOARD_BUTTON_GPIO GPIO_NUM_0
#define BOARD_STATUS_LED_GPIO GPIO_NUM_48

/* Set to 1 if the sensor PCB is mounted with channel 0 on the robot's right. */
#define BOARD_SENSOR_REVERSE 0

/* +1 if IN1-high / IN2-low drives that wheel forward. Use -1 if it runs backward. */
#define BOARD_MOTOR_LEFT_POLARITY  1
#define BOARD_MOTOR_RIGHT_POLARITY 1

/* L298N is a slow bipolar bridge. 1 kHz keeps it out of its switching-loss region. */
#define BOARD_PWM_FREQ_HZ 1000
#define BOARD_PWM_BITS 10
#define BOARD_DUTY_MAX (1 << BOARD_PWM_BITS)

/*
 * Command 0 brakes. Any non-zero command is mapped onto
 * [BOARD_MOTOR_START_DUTY, BOARD_DUTY_MAX] so a geared N20 actually starts
 * instead of sitting in the L298N dead zone and making heat.
 */
#define BOARD_MOTOR_START_DUTY 200
#define BOARD_MOTOR_CMD_DEADBAND 30

#define BOARD_CONTROL_PERIOD_MS 10
#define BOARD_ADC_SAMPLES 8

/* Normalized blackness (0 white … 1000 black) below this does not vote. */
#define BOARD_BLACK_THRESHOLD 180

/* Minimum |black - white| raw counts accepted for one sensor during calibration. */
#define BOARD_CAL_MIN_SPAN 300

#define BOARD_BASE_DUTY 480.0f
#define BOARD_MAX_DUTY ((float)BOARD_DUTY_MAX)
#define BOARD_REVERSE_DUTY 280.0f
#define BOARD_KP 780.0f
#define BOARD_KD 12.0f
#define BOARD_KI 40.0f
#define BOARD_SLOWDOWN 0.55f
#define BOARD_I_LIMIT 0.40f
#define BOARD_SEARCH_DUTY 400.0f
#define BOARD_DERIV_FILTER 0.35f
#define BOARD_GRACE_MS 160
#define BOARD_LOST_TIMEOUT_MS 1400

#define BOARD_BUTTON_SHORT_MS 50
#define BOARD_BUTTON_LONG_MS 1200
#define BOARD_RECAL_WINDOW_MS 3000

/*
 * Optional pack measurement. Default off so a floating GPIO cannot look like
 * a dead battery. When enabled, wire a divider from battery + to GPIO14:
 * 22k from pack to the pin, 10k from the pin to GND. 8.4 V then sits near 2.6 V.
 */
#define BOARD_BATTERY_SENSE_ENABLE 0
#define BOARD_VBAT_GPIO GPIO_NUM_14
#define BOARD_VBAT_R_TOP 22000
#define BOARD_VBAT_R_BOT 10000
#define BOARD_VBAT_CUTOFF_MV 6600
#define BOARD_VBAT_LOW_SAMPLES 5

#define BOARD_STATUS_LED_ENABLE 1

/* ---- line_math.h ---- */

#include <stdbool.h>

/*
 * Pure line-following math. This file does not include ESP-IDF headers so the
 * same object can be compiled and tested on the host.
 *
 * Five sensors are indexed left to right. Blackness is 0 (paper) … 1000 (tape).
 * Position is 0 at the far-left sensor and 4000 at the far-right sensor.
 * Error is (position - 2000) / 2000: positive when the tape is to the right.
 */

#define LINE_SENSOR_COUNT 5
#define LINE_VALUE_MAX 1000
#define LINE_POS_WEIGHT 1000
#define LINE_POS_MAX ((LINE_SENSOR_COUNT - 1) * LINE_POS_WEIGHT)
#define LINE_POS_CENTER (LINE_POS_MAX / 2)

typedef enum {
    LINE_TRACK_FOLLOW = 0,
    LINE_TRACK_GRACE,
    LINE_TRACK_SEARCH,
    LINE_TRACK_STOPPED,
    LINE_TRACK_PAUSED,
} line_track_state_t;

typedef struct {
    float base_duty;
    float max_duty;
    float reverse_duty;
    float kp;
    float kd;
    float ki;
    float slowdown;
    float i_limit;
    float search_duty;
    float deriv_filter;
    int grace_ms;
    int timeout_ms;
} tracker_cfg_t;

typedef struct {
    line_track_state_t state;
    bool paused;
    float error;
    float last_error;
    float d_filt;
    float integral;
    float left;
    float right;
    float hold_left;
    float hold_right;
    int position;
    int last_sign;
    int lost_ms;
    bool have_prev;
} tracker_t;

int line_normalize(int raw, int white, int black);

bool line_centroid(const int values[LINE_SENSOR_COUNT], int threshold,
                   int *position, int *total);

/* Returns true when an output was clamped to the duty limits. */
bool line_mix_duties(const tracker_cfg_t *cfg, float error, float d_error,
                     float integral, float *left, float *right);

void tracker_init(tracker_t *tr);
void tracker_set_paused(tracker_t *tr, bool paused);
void tracker_step(tracker_t *tr, const tracker_cfg_t *cfg,
                  bool line_seen, int position, float dt_s);

const char *line_track_state_name(line_track_state_t state);

/* ---- line_sensors.h ---- */

#include <stdbool.h>

#include "esp_err.h"

/* Calibration is stored in physical channel order: index 0 is SENSOR0's GPIO. */
typedef struct {
    int white[LINE_SENSOR_COUNT];
    int black[LINE_SENSOR_COUNT];
} sensor_cal_t;

/* raw[] and value[] are left-to-right after BOARD_SENSOR_REVERSE is applied. */
typedef struct {
    int raw[LINE_SENSOR_COUNT];
    int value[LINE_SENSOR_COUNT];
    int position;
    int total;
    bool seen;
} line_reading_t;

esp_err_t line_sensors_init(void);
void line_sensors_read_raw(int raw[LINE_SENSOR_COUNT]);
void line_sensors_set_calibration(const sensor_cal_t *cal);
void line_sensors_get_calibration(sensor_cal_t *cal);
bool line_sensors_cal_valid(const sensor_cal_t *cal);
void line_sensors_read(line_reading_t *out);
int line_sensors_gpio(int physical_index);

/* ---- motors.h ---- */

#include "esp_err.h"

/* Signed command is -BOARD_DUTY_MAX … BOARD_DUTY_MAX. Positive is forward. 0 brakes. */
esp_err_t motors_init(void);
void motors_set_signed(int left, int right);
void motors_brake(void);

/* ---- button.h ---- */

#include <stdbool.h>

typedef enum {
    BUTTON_NONE = 0,
    BUTTON_SHORT,
    BUTTON_LONG,
} button_event_t;

void button_init(void);
bool button_is_down(void);
button_event_t button_poll(int dt_ms);
void button_poll_reset(void);

/* ---- status_led.h ---- */

#include <stdint.h>

void status_led_init(void);
void status_led_set(uint8_t red, uint8_t green, uint8_t blue);

/* ---- power_sense.h ---- */

#include <stdbool.h>

void power_sense_init(void);
void power_sense_poll(void);
int power_sense_mv(void);
bool power_sense_low(void);

/* ---- calibration_store.h ---- */

#include "esp_err.h"

bool calibration_store_load(sensor_cal_t *cal);
esp_err_t calibration_store_save(const sensor_cal_t *cal);

/* Only the operator tunables are stored. Timing limits stay with the firmware build. */
bool calibration_store_load_gains(tracker_cfg_t *cfg);
esp_err_t calibration_store_save_gains(const tracker_cfg_t *cfg);
esp_err_t calibration_store_erase_gains(void);

/* ---- calibrate.h ---- */

#include "esp_err.h"

/* Blocks until every sensor has a usable black/white span. Applies and stores it. */
esp_err_t calibrate_interactive(void);

/* ---- follower.h ---- */

#include <stdbool.h>

typedef enum {
    FOLLOW_CMD_TOGGLE = 1,
    FOLLOW_CMD_PAUSE,
    FOLLOW_CMD_RESUME,
    FOLLOW_CMD_CAL,
    FOLLOW_CMD_NUDGE,
    FOLLOW_CMD_LOG_ON,
    FOLLOW_CMD_LOG_OFF,
} follow_cmd_t;

typedef struct {
    int raw[LINE_SENSOR_COUNT];
    int value[LINE_SENSOR_COUNT];
    int position;
    bool seen;
    float error;
    float left;
    float right;
    line_track_state_t state;
    bool low_battery;
    int vbat_mv;
    float kp;
    float kd;
    float ki;
    float base_duty;
    float slowdown;
    bool logging;
} follower_snapshot_t;

void follower_start(void);
void follower_submit(follow_cmd_t cmd);
void follower_set_base_duty(float duty);
void follower_set_kp(float kp);
void follower_set_kd(float kd);
void follower_set_ki(float ki);
void follower_set_slowdown(float slowdown);
void follower_save_gains(void);
void follower_reset_gains(void);
bool follower_copy_snapshot(follower_snapshot_t *out);

/* ---- console_ctrl.h ---- */

void console_start(void);

/* ---- line_math.c ---- */
/* renamed statics: clampf -> line_math_clampf */

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

int line_normalize(int raw, int white, int black)
{
    int span = black - white;
    if (span == 0) {
        return 0;
    }

    int64_t value = ((int64_t)(raw - white) * LINE_VALUE_MAX) / span;
    if (value < 0) {
        return 0;
    }
    if (value > LINE_VALUE_MAX) {
        return LINE_VALUE_MAX;
    }
    return (int)value;
}

bool line_centroid(const int values[LINE_SENSOR_COUNT], int threshold,
                   int *position, int *total)
{
    int32_t weighted = 0;
    int sum = 0;

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        int value = values[i];
        if (value < 0) {
            value = 0;
        } else if (value > LINE_VALUE_MAX) {
            value = LINE_VALUE_MAX;
        }
        if (value < threshold) {
            continue;
        }
        sum += value;
        weighted += (int32_t)value * (int32_t)(i * LINE_POS_WEIGHT);
    }

    if (total != NULL) {
        *total = sum;
    }
    if (sum <= 0) {
        if (position != NULL) {
            *position = LINE_POS_CENTER;
        }
        return false;
    }
    if (position != NULL) {
        *position = (int)(weighted / sum);
    }
    return true;
}

static float line_math_clampf(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

/* Straight: full cruise. Gentle curve: bleed speed. Sharp curve: crawl, so the inner wheel can reverse. */
static float forward_keep(float abs_error, float slowdown)
{
    abs_error = line_math_clampf(abs_error, 0.0f, 1.0f);
    float keep = 1.0f - slowdown * abs_error;
    const float sharp_at = 0.82f;
    if (abs_error > sharp_at) {
        float t = (abs_error - sharp_at) / (1.0f - sharp_at);
        keep = keep * (1.0f - t) + 0.12f * t;
    }
    if (keep < 0.0f) {
        return 0.0f;
    }
    return keep;
}

bool line_mix_duties(const tracker_cfg_t *cfg, float error, float d_error,
                     float integral, float *left, float *right)
{
    float abs_error = fabsf(error);
    float base = cfg->base_duty * forward_keep(abs_error, cfg->slowdown);
    if (base < 0.0f) {
        base = 0.0f;
    }

    float turn = cfg->kp * error + cfg->kd * d_error + cfg->ki * integral;
    float raw_left = base + turn;
    float raw_right = base - turn;
    float out_left = line_math_clampf(raw_left, -cfg->reverse_duty, cfg->max_duty);
    float out_right = line_math_clampf(raw_right, -cfg->reverse_duty, cfg->max_duty);

    if (left != NULL) {
        *left = out_left;
    }
    if (right != NULL) {
        *right = out_right;
    }
    return out_left != raw_left || out_right != raw_right;
}

void tracker_init(tracker_t *tr)
{
    memset(tr, 0, sizeof(*tr));
    tr->last_sign = 1;
    tr->paused = true;
    tr->state = LINE_TRACK_PAUSED;
    tr->position = LINE_POS_CENTER;
}

void tracker_set_paused(tracker_t *tr, bool paused)
{
    if (!paused) {
        tr->paused = false;
        tr->have_prev = false;
        tr->d_filt = 0.0f;
        tr->integral = 0.0f;
        tr->lost_ms = 0;
        if (tr->state == LINE_TRACK_STOPPED || tr->state == LINE_TRACK_PAUSED) {
            tr->state = LINE_TRACK_FOLLOW;
        }
        return;
    }

    tr->paused = true;
    tr->left = 0.0f;
    tr->right = 0.0f;
    if (tr->state != LINE_TRACK_STOPPED) {
        tr->state = LINE_TRACK_PAUSED;
    }
}

static float clamp_error(int position)
{
    float error = (float)(position - LINE_POS_CENTER) / (float)LINE_POS_CENTER;
    return line_math_clampf(error, -1.0f, 1.0f);
}

void tracker_step(tracker_t *tr, const tracker_cfg_t *cfg,
                  bool line_seen, int position, float dt_s)
{
    if (tr->paused || tr->state == LINE_TRACK_STOPPED) {
        tr->left = 0.0f;
        tr->right = 0.0f;
        return;
    }

    if (dt_s < 0.001f) {
        dt_s = 0.001f;
    }
    int dt_ms = (int)(dt_s * 1000.0f + 0.5f);
    if (dt_ms < 1) {
        dt_ms = 1;
    }

    if (line_seen) {
        float error = clamp_error(position);
        float derivative = 0.0f;
        if (tr->have_prev &&
            (tr->state == LINE_TRACK_FOLLOW || tr->state == LINE_TRACK_GRACE)) {
            derivative = (error - tr->last_error) / dt_s;
        }
        float filter = cfg->deriv_filter;
        if (filter < 0.0f) {
            filter = 0.0f;
        } else if (filter > 1.0f) {
            filter = 1.0f;
        }
        tr->d_filt += filter * (derivative - tr->d_filt);

        float saved_integral = tr->integral;
        if (fabsf(error) < 0.55f) {
            tr->integral += error * dt_s;
            tr->integral = line_math_clampf(tr->integral, -cfg->i_limit, cfg->i_limit);
        } else {
            tr->integral *= 0.85f;
        }

        float left = 0.0f;
        float right = 0.0f;
        bool saturated = line_mix_duties(cfg, error, tr->d_filt, tr->integral, &left, &right);
        if (saturated) {
            tr->integral = saved_integral;
            line_mix_duties(cfg, error, tr->d_filt, tr->integral, &left, &right);
        }

        tr->left = left;
        tr->right = right;
        tr->hold_left = left;
        tr->hold_right = right;
        tr->last_error = error;
        tr->error = error;
        tr->position = position;
        tr->have_prev = true;
        tr->lost_ms = 0;
        tr->state = LINE_TRACK_FOLLOW;
        if (error > 0.05f) {
            tr->last_sign = 1;
        } else if (error < -0.05f) {
            tr->last_sign = -1;
        }
        return;
    }

    tr->integral = 0.0f;
    tr->lost_ms += dt_ms;
    if (tr->lost_ms > cfg->timeout_ms + dt_ms) {
        tr->lost_ms = cfg->timeout_ms + dt_ms;
    }

    if (tr->lost_ms <= cfg->grace_ms) {
        tr->state = LINE_TRACK_GRACE;
        tr->left = tr->hold_left;
        tr->right = tr->hold_right;
        tr->error = tr->last_error;
        return;
    }

    if (tr->lost_ms <= cfg->timeout_ms) {
        tr->state = LINE_TRACK_SEARCH;
        tr->have_prev = false;
        tr->d_filt = 0.0f;
        float spin = cfg->search_duty;
        if (spin > cfg->max_duty) {
            spin = cfg->max_duty;
        }
        if (spin < 0.0f) {
            spin = 0.0f;
        }
        int sign = tr->last_sign >= 0 ? 1 : -1;
        tr->left = sign * spin;
        tr->right = -sign * spin;
        tr->error = tr->last_error;
        return;
    }

    tr->state = LINE_TRACK_STOPPED;
    tr->left = 0.0f;
    tr->right = 0.0f;
}

const char *line_track_state_name(line_track_state_t state)
{
    switch (state) {
    case LINE_TRACK_FOLLOW:
        return "FOLLOW";
    case LINE_TRACK_GRACE:
        return "GRACE";
    case LINE_TRACK_SEARCH:
        return "SEARCH";
    case LINE_TRACK_STOPPED:
        return "STOPPED";
    case LINE_TRACK_PAUSED:
        return "PAUSED";
    default:
        return "UNKNOWN";
    }
}

/* ---- line_sensors.c ---- */
/* renamed statics: TAG -> line_sensors_TAG, s_adc -> line_sensors_s_adc, s_channel -> line_sensors_s_channel */

#include <stdlib.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *line_sensors_TAG = "sensors";

_Static_assert(BOARD_SENSOR_COUNT == LINE_SENSOR_COUNT, "sensor count mismatch");

static const gpio_num_t s_gpio[LINE_SENSOR_COUNT] = {
    BOARD_SENSOR0_GPIO,
    BOARD_SENSOR1_GPIO,
    BOARD_SENSOR2_GPIO,
    BOARD_SENSOR3_GPIO,
    BOARD_SENSOR4_GPIO,
};

static adc_oneshot_unit_handle_t line_sensors_s_adc;
static adc_channel_t line_sensors_s_channel[LINE_SENSOR_COUNT];
static sensor_cal_t s_cal;

static int average_channel(adc_channel_t channel)
{
    int sum = 0;
    for (int i = 0; i < BOARD_ADC_SAMPLES; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(line_sensors_s_adc, channel, &raw);
        if (err != ESP_OK) {
            ESP_LOGW(line_sensors_TAG, "adc read failed: %s", esp_err_to_name(err));
            return sum / (i == 0 ? 1 : i);
        }
        sum += raw;
    }
    return sum / BOARD_ADC_SAMPLES;
}

esp_err_t line_sensors_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &line_sensors_s_adc), line_sensors_TAG, "adc unit");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        adc_unit_t unit = ADC_UNIT_1;
        adc_channel_t channel = ADC_CHANNEL_0;
        ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(s_gpio[i], &unit, &channel),
                            line_sensors_TAG, "gpio %d is not an adc pin", s_gpio[i]);
        if (unit != ADC_UNIT_1) {
            ESP_LOGE(line_sensors_TAG, "gpio %d is not on ADC1", s_gpio[i]);
            return ESP_ERR_INVALID_ARG;
        }
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(line_sensors_s_adc, channel, &chan_cfg),
                            line_sensors_TAG, "config gpio %d", s_gpio[i]);
        line_sensors_s_channel[i] = channel;
        ESP_LOGI(line_sensors_TAG, "sensor %d -> GPIO %d (ADC1 ch %d)", i, s_gpio[i], channel);
    }

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        s_cal.white[i] = 700;
        s_cal.black[i] = 2400;
    }

    int discard[LINE_SENSOR_COUNT];
    for (int n = 0; n < 4; n++) {
        line_sensors_read_raw(discard);
    }
    return ESP_OK;
}

void line_sensors_read_raw(int raw[LINE_SENSOR_COUNT])
{
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        raw[i] = (line_sensors_s_adc != NULL) ? average_channel(line_sensors_s_channel[i]) : 0;
    }
}

void line_sensors_set_calibration(const sensor_cal_t *cal)
{
    if (cal != NULL) {
        s_cal = *cal;
    }
}

void line_sensors_get_calibration(sensor_cal_t *cal)
{
    if (cal != NULL) {
        *cal = s_cal;
    }
}

bool line_sensors_cal_valid(const sensor_cal_t *cal)
{
    if (cal == NULL) {
        return false;
    }
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        int span = abs(cal->black[i] - cal->white[i]);
        if (span < BOARD_CAL_MIN_SPAN) {
            return false;
        }
        if (cal->white[i] < 0 || cal->black[i] < 0 || cal->white[i] > 4095 || cal->black[i] > 4095) {
            return false;
        }
    }
    return true;
}

int line_sensors_gpio(int physical_index)
{
    if (physical_index < 0 || physical_index >= LINE_SENSOR_COUNT) {
        return -1;
    }
    return (int)s_gpio[physical_index];
}

void line_sensors_read(line_reading_t *out)
{
    if (out == NULL) {
        return;
    }

    int physical[LINE_SENSOR_COUNT] = {0};
    line_sensors_read_raw(physical);

    for (int logical = 0; logical < LINE_SENSOR_COUNT; logical++) {
        int physical_index = logical;
#if BOARD_SENSOR_REVERSE
        physical_index = LINE_SENSOR_COUNT - 1 - logical;
#endif
        out->raw[logical] = physical[physical_index];
        out->value[logical] = line_normalize(physical[physical_index],
                                             s_cal.white[physical_index],
                                             s_cal.black[physical_index]);
    }

    out->seen = line_centroid(out->value, BOARD_BLACK_THRESHOLD, &out->position, &out->total);
}

/* ---- motors.c ---- */
/* renamed statics: TAG -> motors_TAG, s_ready -> motors_s_ready */

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *motors_TAG = "motors";

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

static bool motors_s_ready;
static bool s_duty_warned;

static void set_enable(ledc_channel_t channel, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(MOTOR_MODE, channel, duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(MOTOR_MODE, channel);
    }
    if (err != ESP_OK && !s_duty_warned) {
        s_duty_warned = true;
        ESP_LOGW(motors_TAG, "pwm update failed: %s", esp_err_to_name(err));
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
    ESP_RETURN_ON_ERROR(gpio_config(&io), motors_TAG, "direction gpios");
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
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), motors_TAG, "pwm timer");

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
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), motors_TAG, "pwm channel");
    }

    motors_s_ready = true;
    motors_brake();
    ESP_LOGI(motors_TAG, "L298N pwm %d Hz, %d-bit, start duty %d",
             BOARD_PWM_FREQ_HZ, BOARD_PWM_BITS, BOARD_MOTOR_START_DUTY);
    return ESP_OK;
}

void motors_set_signed(int left, int right)
{
    if (!motors_s_ready) {
        return;
    }
    drive_side(&s_left, clamp_command(left) * s_left.polarity);
    drive_side(&s_right, clamp_command(right) * s_right.polarity);
}

void motors_brake(void)
{
    if (!motors_s_ready) {
        return;
    }
    brake_side(&s_left);
    brake_side(&s_right);
}

/* ---- button.c ---- */
/* renamed statics: TAG -> button_TAG */

#include "driver/gpio.h"
#include "esp_log.h"

static const char *button_TAG = "button";

static bool s_prev_down;
static int s_held_ms;
static bool s_long_sent;

void button_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BOARD_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_LOGI(button_TAG, "BOOT button on GPIO %d (short pause/go, hold %d ms to calibrate)",
             BOARD_BUTTON_GPIO, BOARD_BUTTON_LONG_MS);
}

bool button_is_down(void)
{
    return gpio_get_level(BOARD_BUTTON_GPIO) == 0;
}

void button_poll_reset(void)
{
    s_prev_down = button_is_down();
    s_held_ms = 0;
    s_long_sent = false;
}

button_event_t button_poll(int dt_ms)
{
    bool down = button_is_down();
    button_event_t event = BUTTON_NONE;

    if (down) {
        if (!s_prev_down) {
            s_held_ms = 0;
            s_long_sent = false;
        }
        s_held_ms += dt_ms;
        if (!s_long_sent && s_held_ms >= BOARD_BUTTON_LONG_MS) {
            s_long_sent = true;
            event = BUTTON_LONG;
        }
    } else if (s_prev_down) {
        if (!s_long_sent && s_held_ms >= BOARD_BUTTON_SHORT_MS) {
            event = BUTTON_SHORT;
        }
        s_held_ms = 0;
        s_long_sent = false;
    }

    s_prev_down = down;
    return event;
}

/* ---- status_led.c ---- */
/* renamed statics: TAG -> status_led_TAG, s_channel -> status_led_s_channel, s_ready -> status_led_s_ready */

#include "esp_log.h"

#if BOARD_STATUS_LED_ENABLE

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

static const char *status_led_TAG = "led";

static rmt_channel_handle_t status_led_s_channel;
static rmt_encoder_handle_t s_encoder;
static uint8_t s_last[3];
static bool s_have_last;
static bool status_led_s_ready;

void status_led_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = BOARD_STATUS_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 48,
        .trans_queue_depth = 4,
    };
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &status_led_s_channel);
    if (err != ESP_OK) {
        ESP_LOGW(status_led_TAG, "status LED channel failed: %s", esp_err_to_name(err));
        return;
    }

    /* 10 MHz ticks. A 0 bit is 0.4 us high / 0.8 us low. A 1 bit is the swap. WS2812 is GRB. */
    rmt_bytes_encoder_config_t encoder_cfg = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 4,
            .level1 = 0,
            .duration1 = 8,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 8,
            .level1 = 0,
            .duration1 = 4,
        },
        .flags.msb_first = 1,
    };
    err = rmt_new_bytes_encoder(&encoder_cfg, &s_encoder);
    if (err != ESP_OK) {
        ESP_LOGW(status_led_TAG, "status LED encoder failed: %s", esp_err_to_name(err));
        return;
    }
    err = rmt_enable(status_led_s_channel);
    if (err != ESP_OK) {
        ESP_LOGW(status_led_TAG, "status LED enable failed: %s", esp_err_to_name(err));
        return;
    }

    status_led_s_ready = true;
    status_led_set(0, 0, 16);
    ESP_LOGI(status_led_TAG, "onboard WS2812 on GPIO %d", BOARD_STATUS_LED_GPIO);
}

void status_led_set(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!status_led_s_ready) {
        return;
    }
    uint8_t grb[3] = {green, red, blue};
    if (s_have_last && grb[0] == s_last[0] && grb[1] == s_last[1] && grb[2] == s_last[2]) {
        return;
    }

    rmt_transmit_config_t tx = {
        .loop_count = 0,
        .flags.eot_level = 0,
    };
    rmt_encoder_reset(s_encoder);
    esp_err_t err = rmt_transmit(status_led_s_channel, s_encoder, grb, sizeof(grb), &tx);
    if (err == ESP_OK) {
        err = rmt_tx_wait_all_done(status_led_s_channel, 20);
    }
    if (err != ESP_OK) {
        ESP_LOGW(status_led_TAG, "status LED write failed: %s", esp_err_to_name(err));
        return;
    }
    s_last[0] = grb[0];
    s_last[1] = grb[1];
    s_last[2] = grb[2];
    s_have_last = true;
}

#else

void status_led_init(void)
{
}

void status_led_set(uint8_t red, uint8_t green, uint8_t blue)
{
    (void)red;
    (void)green;
    (void)blue;
}

#endif

/* ---- power_sense.c ---- */
/* renamed statics: TAG -> power_sense_TAG, s_adc -> power_sense_s_adc, s_channel -> power_sense_s_channel, s_ready -> power_sense_s_ready */

#include "esp_log.h"

static const char *power_sense_TAG = "power";

#if BOARD_BATTERY_SENSE_ENABLE

#include <stdint.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

static adc_oneshot_unit_handle_t power_sense_s_adc;
static adc_cali_handle_t s_cali;
static adc_channel_t power_sense_s_channel;
static bool power_sense_s_ready;
static bool s_have_cali;
static int s_mv = -1;
static int s_low_count;
static int s_prescale;

static int pin_millivolts(int raw)
{
    int mv = 0;
    if (s_have_cali && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
        return mv;
    }
    return (raw * 3100) / 4095;
}

void power_sense_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &power_sense_s_adc) != ESP_OK) {
        ESP_LOGW(power_sense_TAG, "battery ADC unit failed");
        return;
    }

    adc_unit_t unit = ADC_UNIT_2;
    if (adc_oneshot_io_to_channel(BOARD_VBAT_GPIO, &unit, &power_sense_s_channel) != ESP_OK || unit != ADC_UNIT_2) {
        ESP_LOGW(power_sense_TAG, "GPIO %d is not an ADC2 battery pin", BOARD_VBAT_GPIO);
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    if (adc_oneshot_config_channel(power_sense_s_adc, power_sense_s_channel, &chan_cfg) != ESP_OK) {
        ESP_LOGW(power_sense_TAG, "battery channel config failed");
        return;
    }

    adc_cali_curve_fitting_config_t cal_cfg = {
        .unit_id = ADC_UNIT_2,
        .chan = power_sense_s_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal_cfg, &s_cali) == ESP_OK) {
        s_have_cali = true;
    } else {
        ESP_LOGW(power_sense_TAG, "battery ADC calibration unavailable; cutoff is approximate");
    }

    power_sense_s_ready = true;
    ESP_LOGI(power_sense_TAG, "pack sense on GPIO %d, cutoff %d mV, divider %d/%d",
             BOARD_VBAT_GPIO, BOARD_VBAT_CUTOFF_MV, BOARD_VBAT_R_TOP, BOARD_VBAT_R_BOT);
}

void power_sense_poll(void)
{
    if (!power_sense_s_ready) {
        return;
    }
    if (++s_prescale < 10) {
        return;
    }
    s_prescale = 0;

    int raw = 0;
    if (adc_oneshot_read(power_sense_s_adc, power_sense_s_channel, &raw) != ESP_OK) {
        return;
    }
    int pin_mv = pin_millivolts(raw);
    int pack_mv = (int)(((int64_t)pin_mv * (BOARD_VBAT_R_TOP + BOARD_VBAT_R_BOT)) / BOARD_VBAT_R_BOT);
    s_mv = pack_mv;
    if (pack_mv < BOARD_VBAT_CUTOFF_MV) {
        if (s_low_count < 1000) {
            s_low_count++;
        }
    } else {
        s_low_count = 0;
    }
}

int power_sense_mv(void)
{
    return s_mv;
}

bool power_sense_low(void)
{
    return power_sense_s_ready && s_low_count >= BOARD_VBAT_LOW_SAMPLES;
}

#else

void power_sense_init(void)
{
    ESP_LOGI(power_sense_TAG, "pack sensing off (BOARD_BATTERY_SENSE_ENABLE is 0)");
}

void power_sense_poll(void)
{
}

int power_sense_mv(void)
{
    return -1;
}

bool power_sense_low(void)
{
    return false;
}

#endif

/* ---- calibration_store.c ---- */
/* renamed statics: TAG -> calibration_store_TAG */

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "nvs.h"

static const char *calibration_store_TAG = "store";
static const char *NVS_NS = "follower";

#define CAL_MAGIC 0x314C4143u
#define GAIN_MAGIC 0x314E4147u

typedef struct {
    uint32_t magic;
    int32_t white[LINE_SENSOR_COUNT];
    int32_t black[LINE_SENSOR_COUNT];
} cal_blob_t;

typedef struct {
    uint32_t magic;
    float base_duty;
    float kp;
    float kd;
    float ki;
    float slowdown;
} gain_blob_t;

bool calibration_store_load(sensor_cal_t *cal)
{
    if (cal == NULL) {
        return false;
    }

    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    cal_blob_t blob;
    size_t size = sizeof(blob);
    esp_err_t err = nvs_get_blob(handle, "cal", &blob, &size);
    nvs_close(handle);
    if (err != ESP_OK || size != sizeof(blob) || blob.magic != CAL_MAGIC) {
        return false;
    }

    sensor_cal_t loaded;
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        loaded.white[i] = (int)blob.white[i];
        loaded.black[i] = (int)blob.black[i];
    }
    if (!line_sensors_cal_valid(&loaded)) {
        ESP_LOGW(calibration_store_TAG, "stored calibration failed the span check");
        return false;
    }

    *cal = loaded;
    return true;
}

esp_err_t calibration_store_save(const sensor_cal_t *cal)
{
    if (cal == NULL || !line_sensors_cal_valid(cal)) {
        return ESP_ERR_INVALID_ARG;
    }

    cal_blob_t blob = {.magic = CAL_MAGIC};
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        blob.white[i] = (int32_t)cal->white[i];
        blob.black[i] = (int32_t)cal->black[i];
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, "cal", &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err == ESP_OK) {
        ESP_LOGI(calibration_store_TAG, "calibration saved");
    }
    return err;
}

bool calibration_store_load_gains(tracker_cfg_t *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    gain_blob_t blob;
    size_t size = sizeof(blob);
    esp_err_t err = nvs_get_blob(handle, "gains", &blob, &size);
    nvs_close(handle);
    if (err != ESP_OK || size != sizeof(blob) || blob.magic != GAIN_MAGIC) {
        return false;
    }
    if (!isfinite(blob.base_duty) || blob.base_duty < 0.0f || blob.base_duty > 1024.0f ||
        !isfinite(blob.kp) || blob.kp < 0.0f || blob.kp > 4000.0f ||
        !isfinite(blob.kd) || blob.kd < 0.0f || blob.kd > 400.0f ||
        !isfinite(blob.ki) || blob.ki < 0.0f || blob.ki > 2000.0f ||
        !isfinite(blob.slowdown) || blob.slowdown < 0.0f || blob.slowdown > 1.0f) {
        ESP_LOGW(calibration_store_TAG, "stored gains were out of range");
        return false;
    }

    cfg->base_duty = blob.base_duty;
    cfg->kp = blob.kp;
    cfg->kd = blob.kd;
    cfg->ki = blob.ki;
    cfg->slowdown = blob.slowdown;
    ESP_LOGI(calibration_store_TAG, "stored gains loaded");
    return true;
}

esp_err_t calibration_store_save_gains(const tracker_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gain_blob_t blob = {
        .magic = GAIN_MAGIC,
        .base_duty = cfg->base_duty,
        .kp = cfg->kp,
        .kd = cfg->kd,
        .ki = cfg->ki,
        .slowdown = cfg->slowdown,
    };

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, "gains", &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err == ESP_OK) {
        ESP_LOGI(calibration_store_TAG, "gains saved");
    }
    return err;
}

esp_err_t calibration_store_erase_gains(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(handle, "gains");
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

/* ---- calibrate.c ---- */
/* renamed statics: TAG -> calibrate_TAG */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *calibrate_TAG = "cal";

static const char *k_side[LINE_SENSOR_COUNT] = {
    "far left", "left", "center", "right", "far right",
};

static int logical_index(int physical)
{
#if BOARD_SENSOR_REVERSE
    return LINE_SENSOR_COUNT - 1 - physical;
#else
    return physical;
#endif
}

static void wait_for_press(uint8_t red, uint8_t green, uint8_t blue)
{
    while (button_is_down()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    int ticks = 0;
    for (;;) {
        bool on = ((ticks / 15) % 2) == 0;
        status_led_set(on ? red : 0, on ? green : 0, on ? blue : 0);
        if (button_is_down()) {
            vTaskDelay(pdMS_TO_TICKS(30));
            if (button_is_down()) {
                return;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ticks++;
    }
}

static void wait_for_release(void)
{
    while (button_is_down()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(40));
}

static void sample_white(sensor_cal_t *cal)
{
    int accumulator[LINE_SENSOR_COUNT] = {0};
    const int samples = 48;
    for (int n = 0; n < samples; n++) {
        int raw[LINE_SENSOR_COUNT];
        line_sensors_read_raw(raw);
        for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
            accumulator[i] += raw[i];
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        cal->white[i] = accumulator[i] / samples;
    }
}

static bool sample_black(const sensor_cal_t *cal, sensor_cal_t *out)
{
    ESP_LOGI(calibrate_TAG, "Hold BOOT and rock the bar across the tape until every sensor has seen black. Release BOOT to finish.");
    wait_for_press(0, 0, 40);

    enum { k_bins = 16 };
    uint16_t hist[LINE_SENSOR_COUNT][k_bins];
    int min_raw[LINE_SENSOR_COUNT];
    int max_raw[LINE_SENSOR_COUNT];
    memset(hist, 0, sizeof(hist));
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        min_raw[i] = 4096;
        max_raw[i] = 0;
    }

    int64_t started = esp_timer_get_time();
    int samples = 0;
    status_led_set(0, 0, 40);
    while (button_is_down()) {
        int raw[LINE_SENSOR_COUNT];
        line_sensors_read_raw(raw);
        for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
            int sample = raw[i];
            if (sample < min_raw[i]) {
                min_raw[i] = sample;
            }
            if (sample > max_raw[i]) {
                max_raw[i] = sample;
            }
            int bin = sample * k_bins / 4096;
            if (bin >= k_bins) {
                bin = k_bins - 1;
            }
            if (bin < 0) {
                bin = 0;
            }
            if (hist[i][bin] < UINT16_MAX) {
                hist[i][bin]++;
            }
        }
        samples++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    int held_ms = (int)((esp_timer_get_time() - started) / 1000);
    if (held_ms < 400 || samples < 20) {
        ESP_LOGW(calibrate_TAG, "That hold was %d ms. Keep BOOT down while you rock across the line.", held_ms);
        return false;
    }

    bool ok = true;
    *out = *cal;
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        int best_bin = -1;
        int best_dist = -1;
        for (int bin = 0; bin < k_bins; bin++) {
            if (hist[i][bin] < 2) {
                continue;
            }
            int center = (bin * 4096 / k_bins) + (4096 / (2 * k_bins));
            int dist = abs(center - cal->white[i]);
            if (dist > best_dist) {
                best_dist = dist;
                best_bin = bin;
            }
        }

        int black = (best_bin >= 0)
                        ? (best_bin * 4096 / k_bins) + (4096 / (2 * k_bins))
                        : cal->white[i];
        out->black[i] = black;
        int span = abs(black - cal->white[i]);
        const char *sense = black >= cal->white[i] ? "high=black" : "low=black";
        ESP_LOGI(calibrate_TAG, "GPIO %d (%s) white=%d black=%d span=%d %s raw %d..%d",
                 line_sensors_gpio(i), k_side[logical_index(i)], cal->white[i], black, span, sense,
                 min_raw[i], max_raw[i]);
        if (span < BOARD_CAL_MIN_SPAN) {
            ESP_LOGW(calibrate_TAG, "GPIO %d did not see enough contrast. Rock that sensor over the tape.",
                     line_sensors_gpio(i));
            ok = false;
        }
    }
    return ok;
}

esp_err_t calibrate_interactive(void)
{
    sensor_cal_t cal;
    memset(&cal, 0, sizeof(cal));

    ESP_LOGI(calibrate_TAG, "Step 1/2: set all five sensors on bare white track, then press BOOT.");
    wait_for_press(0, 0, 36);
    wait_for_release();
    sample_white(&cal);
    ESP_LOGI(calibrate_TAG, "White raw: %d %d %d %d %d",
             cal.white[0], cal.white[1], cal.white[2], cal.white[3], cal.white[4]);

    for (;;) {
        ESP_LOGI(calibrate_TAG, "Step 2/2: black tape. Index 0 in the log is the sensor on GPIO %d.",
                 line_sensors_gpio(0));
        sensor_cal_t measured;
        if (!sample_black(&cal, &measured)) {
            continue;
        }
        if (!line_sensors_cal_valid(&measured)) {
            continue;
        }
        line_sensors_set_calibration(&measured);
        return calibration_store_save(&measured);
    }
}

/* ---- follower.c ---- */
/* renamed statics: TAG -> follower_TAG, clampf -> follower_clampf, format_fixed -> follower_format_fixed */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *follower_TAG = "follow";

static tracker_t s_tr;
static tracker_cfg_t s_cfg;
static follower_snapshot_t s_snap;
static SemaphoreHandle_t s_mu;
static QueueHandle_t s_cmds;
static bool s_logging = true;
static bool s_low_latched;
static int64_t s_last_us;
static TickType_t s_wake;

static float follower_clampf(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static void follower_format_fixed(char *dst, size_t len, float value)
{
    int milli = (int)(value >= 0.0f ? value * 1000.0f + 0.5f : value * 1000.0f - 0.5f);
    int mag = milli < 0 ? -milli : milli;
    snprintf(dst, len, "%s%d.%03d", milli < 0 ? "-" : "", mag / 1000, mag % 1000);
}

static void default_cfg(tracker_cfg_t *cfg)
{
    *cfg = (tracker_cfg_t){
        .base_duty = BOARD_BASE_DUTY,
        .max_duty = BOARD_MAX_DUTY,
        .reverse_duty = BOARD_REVERSE_DUTY,
        .kp = BOARD_KP,
        .kd = BOARD_KD,
        .ki = BOARD_KI,
        .slowdown = BOARD_SLOWDOWN,
        .i_limit = BOARD_I_LIMIT,
        .search_duty = BOARD_SEARCH_DUTY,
        .deriv_filter = BOARD_DERIV_FILTER,
        .grace_ms = BOARD_GRACE_MS,
        .timeout_ms = BOARD_LOST_TIMEOUT_MS,
    };
}

static void lock(void)
{
    if (s_mu != NULL) {
        xSemaphoreTake(s_mu, portMAX_DELAY);
    }
}

static void unlock(void)
{
    if (s_mu != NULL) {
        xSemaphoreGive(s_mu);
    }
}

static int round_to_int(float value)
{
    return (int)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static void paint_led(void)
{
    if (s_low_latched) {
        status_led_set(48, 0, 0);
        return;
    }
    switch (s_tr.state) {
    case LINE_TRACK_FOLLOW:
        status_led_set(0, 36, 0);
        break;
    case LINE_TRACK_GRACE:
        status_led_set(28, 22, 0);
        break;
    case LINE_TRACK_SEARCH:
        status_led_set(42, 12, 0);
        break;
    case LINE_TRACK_STOPPED:
        status_led_set(42, 0, 0);
        break;
    case LINE_TRACK_PAUSED:
    default:
        status_led_set(16, 0, 28);
        break;
    }
}

static void publish(const line_reading_t *reading, float dt_error)
{
    follower_snapshot_t snap = {0};
    memcpy(snap.raw, reading->raw, sizeof(snap.raw));
    memcpy(snap.value, reading->value, sizeof(snap.value));
    snap.position = reading->position;
    snap.seen = reading->seen;
    snap.error = dt_error;
    snap.left = s_tr.left;
    snap.right = s_tr.right;
    snap.state = s_tr.state;
    snap.low_battery = s_low_latched;
    snap.vbat_mv = power_sense_mv();
    lock();
    snap.kp = s_cfg.kp;
    snap.kd = s_cfg.kd;
    snap.ki = s_cfg.ki;
    snap.base_duty = s_cfg.base_duty;
    snap.slowdown = s_cfg.slowdown;
    snap.logging = s_logging;
    s_snap = snap;
    bool logging = s_logging;
    unlock();

    static int accum_ms;
    accum_ms += BOARD_CONTROL_PERIOD_MS;
    if (!logging || accum_ms < 500) {
        return;
    }
    accum_ms = 0;

    char err_text[16];
    follower_format_fixed(err_text, sizeof(err_text), dt_error);
    ESP_LOGI(follower_TAG, "%s pos=%d err=%s cmd=%d,%d blk=%d %d %d %d %d",
             line_track_state_name(s_tr.state), reading->position, err_text,
             round_to_int(s_tr.left), round_to_int(s_tr.right),
             reading->value[0], reading->value[1], reading->value[2],
             reading->value[3], reading->value[4]);
}

static void arm_clock(void)
{
    s_last_us = esp_timer_get_time();
    s_wake = xTaskGetTickCount();
}

static void run_calibration(void)
{
    motors_brake();
    esp_err_t err = calibrate_interactive();
    if (err != ESP_OK) {
        ESP_LOGE(follower_TAG, "calibration was not stored: %s", esp_err_to_name(err));
    }
    tracker_init(&s_tr);
    button_poll_reset();
    arm_clock();
    ESP_LOGI(follower_TAG, "Paused. Short-press BOOT or type `go` when the robot is on the line.");
}

static bool resume_motion(void)
{
    if (s_low_latched && power_sense_low()) {
        ESP_LOGW(follower_TAG, "Pack is still under %d mV (%d mV). Charge it before driving.",
                 BOARD_VBAT_CUTOFF_MV, power_sense_mv());
        return false;
    }
    s_low_latched = false;
    if (s_tr.paused || s_tr.state == LINE_TRACK_STOPPED || s_tr.state == LINE_TRACK_PAUSED) {
        tracker_set_paused(&s_tr, false);
        ESP_LOGI(follower_TAG, "Running");
    }
    return true;
}

static void pause_motion(void)
{
    if (!s_tr.paused && s_tr.state != LINE_TRACK_STOPPED) {
        tracker_set_paused(&s_tr, true);
        motors_brake();
        ESP_LOGI(follower_TAG, "Paused");
    }
}

static void nudge_motors(void)
{
    bool resume = s_tr.state == LINE_TRACK_FOLLOW || s_tr.state == LINE_TRACK_GRACE ||
                  s_tr.state == LINE_TRACK_SEARCH;
    tracker_set_paused(&s_tr, true);
    motors_brake();
    vTaskDelay(pdMS_TO_TICKS(30));
    ESP_LOGI(follower_TAG, "Nudging both wheels forward for 400 ms. Hold the chassis up.");
    motors_set_signed(360, 360);
    vTaskDelay(pdMS_TO_TICKS(400));
    motors_brake();
    ESP_LOGI(follower_TAG, "Each wheel should have rolled forward. If one ran backward, set that BOARD_MOTOR_*_POLARITY to -1 or swap its two wires.");
    if (resume && !s_low_latched) {
        tracker_set_paused(&s_tr, false);
    }
    arm_clock();
}

static void handle_command(follow_cmd_t cmd)
{
    switch (cmd) {
    case FOLLOW_CMD_TOGGLE:
        if (s_tr.paused || s_tr.state == LINE_TRACK_STOPPED || s_tr.state == LINE_TRACK_PAUSED || s_low_latched) {
            resume_motion();
        } else {
            pause_motion();
        }
        break;
    case FOLLOW_CMD_PAUSE:
        pause_motion();
        break;
    case FOLLOW_CMD_RESUME:
        resume_motion();
        break;
    case FOLLOW_CMD_CAL:
        run_calibration();
        break;
    case FOLLOW_CMD_NUDGE:
        nudge_motors();
        break;
    case FOLLOW_CMD_LOG_ON:
    case FOLLOW_CMD_LOG_OFF:
        lock();
        s_logging = cmd == FOLLOW_CMD_LOG_ON;
        unlock();
        ESP_LOGI(follower_TAG, "live log %s", cmd == FOLLOW_CMD_LOG_ON ? "on" : "off");
        break;
    default:
        break;
    }
}

static void offer_recal_window(void)
{
    ESP_LOGI(follower_TAG, "Stored calibration found. Press BOOT within %d ms to redo it.", BOARD_RECAL_WINDOW_MS);
    int64_t start = esp_timer_get_time();
    int ticks = 0;
    while ((esp_timer_get_time() - start) / 1000 < BOARD_RECAL_WINDOW_MS) {
        bool on = ((ticks / 10) % 2) == 0;
        status_led_set(on ? 28 : 0, on ? 22 : 0, 0);
        if (button_is_down()) {
            vTaskDelay(pdMS_TO_TICKS(30));
            if (button_is_down()) {
                while (button_is_down()) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                run_calibration();
                return;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        ticks++;
    }
}

static void control_task(void *arg)
{
    (void)arg;

    sensor_cal_t stored;
    if (calibration_store_load(&stored)) {
        line_sensors_set_calibration(&stored);
        ESP_LOGI(follower_TAG, "Calibration loaded");
        offer_recal_window();
    } else {
        ESP_LOGW(follower_TAG, "No calibration stored. Starting the white/black capture.");
        run_calibration();
    }

    tracker_init(&s_tr);
    button_poll_reset();
    arm_clock();
    ESP_LOGI(follower_TAG, "Paused with the brake on. Short-press BOOT or type `go` to follow. `help` lists commands.");

    for (;;) {
        follow_cmd_t cmd;
        while (xQueueReceive(s_cmds, &cmd, 0) == pdTRUE) {
            handle_command(cmd);
        }

        button_event_t event = button_poll(BOARD_CONTROL_PERIOD_MS);
        if (event == BUTTON_LONG) {
            run_calibration();
        } else if (event == BUTTON_SHORT) {
            handle_command(FOLLOW_CMD_TOGGLE);
        }

        int64_t now = esp_timer_get_time();
        float dt = (float)(now - s_last_us) / 1000000.0f;
        s_last_us = now;
        if (dt < 0.001f || dt > 0.05f) {
            dt = BOARD_CONTROL_PERIOD_MS / 1000.0f;
        }

        tracker_cfg_t cfg;
        lock();
        cfg = s_cfg;
        unlock();

        line_reading_t reading;
        line_sensors_read(&reading);
        power_sense_poll();

        if (power_sense_low()) {
            if (!s_low_latched) {
                s_low_latched = true;
                tracker_set_paused(&s_tr, true);
                motors_brake();
                ESP_LOGW(follower_TAG, "Pack %d mV is under %d mV. Motors held.",
                         power_sense_mv(), BOARD_VBAT_CUTOFF_MV);
            }
        }

        if (!s_low_latched) {
            tracker_step(&s_tr, &cfg, reading.seen, reading.position, dt);
            motors_set_signed(round_to_int(s_tr.left), round_to_int(s_tr.right));
        } else {
            motors_brake();
        }

        float shown_error = s_tr.error;
        if (reading.seen) {
            shown_error = (float)(reading.position - LINE_POS_CENTER) / (float)LINE_POS_CENTER;
        }
        paint_led();
        publish(&reading, shown_error);

        vTaskDelayUntil(&s_wake, pdMS_TO_TICKS(BOARD_CONTROL_PERIOD_MS));
    }
}

void follower_start(void)
{
    default_cfg(&s_cfg);
    if (!calibration_store_load_gains(&s_cfg)) {
        ESP_LOGI(follower_TAG, "using built-in gains");
    }
    tracker_init(&s_tr);
    s_mu = xSemaphoreCreateMutex();
    s_cmds = xQueueCreate(8, sizeof(follow_cmd_t));
    if (s_mu == NULL || s_cmds == NULL) {
        ESP_LOGE(follower_TAG, "no memory for the follower task");
        return;
    }
    s_snap.state = LINE_TRACK_PAUSED;
    s_snap.logging = true;
    s_snap.vbat_mv = -1;
    if (xTaskCreate(control_task, "follow", 8192, NULL, 8, NULL) != pdPASS) {
        ESP_LOGE(follower_TAG, "follower task was not created");
    }
}

void follower_submit(follow_cmd_t cmd)
{
    if (s_cmds != NULL) {
        xQueueSend(s_cmds, &cmd, 0);
    }
}

static void set_field(float *field, float value, float lo, float hi, const char *name)
{
    float clamped = follower_clampf(value, lo, hi);
    lock();
    *field = clamped;
    unlock();
    char text[16];
    follower_format_fixed(text, sizeof(text), clamped);
    ESP_LOGI(follower_TAG, "%s = %s", name, text);
}

void follower_set_base_duty(float duty)
{
    set_field(&s_cfg.base_duty, duty, 0.0f, BOARD_MAX_DUTY, "speed");
}

void follower_set_kp(float kp)
{
    set_field(&s_cfg.kp, kp, 0.0f, 4000.0f, "kp");
}

void follower_set_kd(float kd)
{
    set_field(&s_cfg.kd, kd, 0.0f, 400.0f, "kd");
}

void follower_set_ki(float ki)
{
    set_field(&s_cfg.ki, ki, 0.0f, 2000.0f, "ki");
}

void follower_set_slowdown(float slowdown)
{
    set_field(&s_cfg.slowdown, slowdown, 0.0f, 1.0f, "slow");
}

void follower_save_gains(void)
{
    tracker_cfg_t cfg;
    lock();
    cfg = s_cfg;
    unlock();
    esp_err_t err = calibration_store_save_gains(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(follower_TAG, "could not save gains: %s", esp_err_to_name(err));
    }
}

void follower_reset_gains(void)
{
    tracker_cfg_t cfg;
    default_cfg(&cfg);
    lock();
    s_cfg.base_duty = cfg.base_duty;
    s_cfg.kp = cfg.kp;
    s_cfg.kd = cfg.kd;
    s_cfg.ki = cfg.ki;
    s_cfg.slowdown = cfg.slowdown;
    unlock();
    esp_err_t err = calibration_store_erase_gains();
    if (err != ESP_OK) {
        ESP_LOGE(follower_TAG, "could not erase gains: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(follower_TAG, "gains restored to the built-in defaults");
    }
}

bool follower_copy_snapshot(follower_snapshot_t *out)
{
    if (out == NULL || s_mu == NULL) {
        return false;
    }
    lock();
    *out = s_snap;
    unlock();
    return true;
}

/* ---- console_ctrl.c ---- */
/* renamed statics: TAG -> console_ctrl_TAG, format_fixed -> console_ctrl_format_fixed */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *console_ctrl_TAG = "cli";

static void console_ctrl_format_fixed(char *dst, size_t len, float value)
{
    int milli = (int)(value >= 0.0f ? value * 1000.0f + 0.5f : value * 1000.0f - 0.5f);
    int mag = milli < 0 ? -milli : milli;
    snprintf(dst, len, "%s%d.%03d", milli < 0 ? "-" : "", mag / 1000, mag % 1000);
}

static void print_help(void)
{
    ESP_LOGI(console_ctrl_TAG, "help                  this list");
    ESP_LOGI(console_ctrl_TAG, "show                  sensors, error, wheel commands");
    ESP_LOGI(console_ctrl_TAG, "pins                  wiring map");
    ESP_LOGI(console_ctrl_TAG, "bat                   pack voltage, if sensing is enabled");
    ESP_LOGI(console_ctrl_TAG, "go / pause / stop     start or brake");
    ESP_LOGI(console_ctrl_TAG, "nudge                 roll both wheels forward for 0.4 s");
    ESP_LOGI(console_ctrl_TAG, "cal                   white/black capture");
    ESP_LOGI(console_ctrl_TAG, "log on|off            2 Hz status line");
    ESP_LOGI(console_ctrl_TAG, "kp|kd|ki|speed|slow   print or set a gain");
    ESP_LOGI(console_ctrl_TAG, "save                  store kp kd ki speed slow");
    ESP_LOGI(console_ctrl_TAG, "defaults              restore those five built-in values");
}

static void print_pins(void)
{
    ESP_LOGI(console_ctrl_TAG, "sensors L->R GPIO %d %d %d %d %d (ADC, 3.3 V)",
             BOARD_SENSOR0_GPIO, BOARD_SENSOR1_GPIO, BOARD_SENSOR2_GPIO,
             BOARD_SENSOR3_GPIO, BOARD_SENSOR4_GPIO);
    ESP_LOGI(console_ctrl_TAG, "left  IN1=%d IN2=%d ENA=%d", BOARD_LEFT_IN1_GPIO, BOARD_LEFT_IN2_GPIO, BOARD_LEFT_EN_GPIO);
    ESP_LOGI(console_ctrl_TAG, "right IN3=%d IN4=%d ENB=%d", BOARD_RIGHT_IN1_GPIO, BOARD_RIGHT_IN2_GPIO, BOARD_RIGHT_EN_GPIO);
    ESP_LOGI(console_ctrl_TAG, "BOOT button GPIO %d, WS2812 GPIO %d", BOARD_BUTTON_GPIO, BOARD_STATUS_LED_GPIO);
    ESP_LOGI(console_ctrl_TAG, "sensor reverse=%d  left polarity=%d  right polarity=%d",
             BOARD_SENSOR_REVERSE, BOARD_MOTOR_LEFT_POLARITY, BOARD_MOTOR_RIGHT_POLARITY);
}

static void print_show(void)
{
    follower_snapshot_t snap;
    if (!follower_copy_snapshot(&snap)) {
        ESP_LOGW(console_ctrl_TAG, "follower is not running");
        return;
    }
    char err_text[16];
    char kp[16];
    char kd[16];
    char ki[16];
    char speed[16];
    char slow[16];
    console_ctrl_format_fixed(err_text, sizeof(err_text), snap.error);
    console_ctrl_format_fixed(kp, sizeof(kp), snap.kp);
    console_ctrl_format_fixed(kd, sizeof(kd), snap.kd);
    console_ctrl_format_fixed(ki, sizeof(ki), snap.ki);
    console_ctrl_format_fixed(speed, sizeof(speed), snap.base_duty);
    console_ctrl_format_fixed(slow, sizeof(slow), snap.slowdown);
    ESP_LOGI(console_ctrl_TAG, "%s pos=%d err=%s seen=%d low=%d bat=%d",
             line_track_state_name(snap.state), snap.position, err_text,
             snap.seen ? 1 : 0, snap.low_battery ? 1 : 0, snap.vbat_mv);
    ESP_LOGI(console_ctrl_TAG, "cmd L=%d R=%d  blk=%d %d %d %d %d",
             (int)(snap.left >= 0 ? snap.left + 0.5f : snap.left - 0.5f),
             (int)(snap.right >= 0 ? snap.right + 0.5f : snap.right - 0.5f),
             snap.value[0], snap.value[1], snap.value[2], snap.value[3], snap.value[4]);
    ESP_LOGI(console_ctrl_TAG, "raw L->R %d %d %d %d %d",
             snap.raw[0], snap.raw[1], snap.raw[2], snap.raw[3], snap.raw[4]);
    ESP_LOGI(console_ctrl_TAG, "speed=%s kp=%s kd=%s ki=%s slow=%s log=%s",
             speed, kp, kd, ki, slow, snap.logging ? "on" : "off");
}

/* Returns 1 when `line` is exactly `name`, 2 when it is `name <float>`, 0 otherwise. */
static int take_float(const char *line, const char *name, float *out)
{
    size_t n = strlen(name);
    if (strncmp(line, name, n) != 0) {
        return 0;
    }
    if (line[n] == '\0') {
        return 1;
    }
    if (line[n] != ' ') {
        return 0;
    }
    char *end = NULL;
    float value = strtof(line + n + 1, &end);
    if (end == line + n + 1) {
        return 0;
    }
    *out = value;
    return 2;
}

static void handle_line(char *line)
{
    while (*line == ' ') {
        line++;
    }
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r' || line[n - 1] == ' ')) {
        line[--n] = '\0';
    }
    if (line[0] == '\0') {
        return;
    }

    float value = 0.0f;
    int kind = 0;
    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        print_help();
    } else if (strcmp(line, "show") == 0) {
        print_show();
    } else if (strcmp(line, "pins") == 0) {
        print_pins();
    } else if (strcmp(line, "bat") == 0) {
        follower_snapshot_t snap;
        if (follower_copy_snapshot(&snap)) {
            if (snap.vbat_mv < 0) {
                ESP_LOGI(console_ctrl_TAG, "pack sensing is compiled out");
            } else {
                ESP_LOGI(console_ctrl_TAG, "pack %d mV", snap.vbat_mv);
            }
        }
    } else if (strcmp(line, "go") == 0) {
        follower_submit(FOLLOW_CMD_RESUME);
    } else if (strcmp(line, "pause") == 0 || strcmp(line, "stop") == 0) {
        follower_submit(FOLLOW_CMD_PAUSE);
    } else if (strcmp(line, "cal") == 0) {
        follower_submit(FOLLOW_CMD_CAL);
    } else if (strcmp(line, "nudge") == 0) {
        follower_submit(FOLLOW_CMD_NUDGE);
    } else if (strcmp(line, "log on") == 0) {
        follower_submit(FOLLOW_CMD_LOG_ON);
    } else if (strcmp(line, "log off") == 0) {
        follower_submit(FOLLOW_CMD_LOG_OFF);
    } else if (strcmp(line, "save") == 0) {
        follower_save_gains();
    } else if (strcmp(line, "defaults") == 0) {
        follower_reset_gains();
    } else if ((kind = take_float(line, "kp", &value)) != 0) {
        if (kind == 1) {
            print_show();
        } else {
            follower_set_kp(value);
        }
    } else if ((kind = take_float(line, "kd", &value)) != 0) {
        if (kind == 1) {
            print_show();
        } else {
            follower_set_kd(value);
        }
    } else if ((kind = take_float(line, "ki", &value)) != 0) {
        if (kind == 1) {
            print_show();
        } else {
            follower_set_ki(value);
        }
    } else if ((kind = take_float(line, "speed", &value)) != 0) {
        if (kind == 1) {
            print_show();
        } else {
            follower_set_base_duty(value);
        }
    } else if ((kind = take_float(line, "slow", &value)) != 0) {
        if (kind == 1) {
            print_show();
        } else {
            follower_set_slowdown(value);
        }
    } else {
        ESP_LOGW(console_ctrl_TAG, "unknown command '%s'", line);
        print_help();
    }
}

static void console_task(void *arg)
{
    (void)arg;
    char line[80];
    print_help();
    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        handle_line(line);
    }
}

void console_start(void)
{
    xTaskCreate(console_task, "console", 6144, NULL, 3, NULL);
}

/* ---- main.c ---- */
/* renamed statics: TAG -> main_TAG */

#include "esp_log.h"
#include "nvs_flash.h"

static const char *main_TAG = "app";

void app_main(void)
{
    /* Brake before flash init so a slow boot cannot leave the L298N enables floating. */
    ESP_ERROR_CHECK(motors_init());
    status_led_init();
    button_init();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(line_sensors_init());
    power_sense_init();

    ESP_LOGI(main_TAG, "ESP32-S3 line follower: 5 analog IR sensors, L298N, two N20 motors");
    follower_start();
    console_start();
}

