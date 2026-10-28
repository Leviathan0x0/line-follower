#include "button.h"

#include "board_config.h"

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "button";

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
    ESP_LOGI(TAG, "BOOT button on GPIO %d (short pause/go, hold %d ms to calibrate)",
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
