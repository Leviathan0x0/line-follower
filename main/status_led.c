#include "status_led.h"

#include "board_config.h"

#include "esp_log.h"

#if BOARD_STATUS_LED_ENABLE

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

static const char *TAG = "led";

static rmt_channel_handle_t s_channel;
static rmt_encoder_handle_t s_encoder;
static uint8_t s_last[3];
static bool s_have_last;
static bool s_ready;

void status_led_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = BOARD_STATUS_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 48,
        .trans_queue_depth = 4,
    };
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &s_channel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "status LED channel failed: %s", esp_err_to_name(err));
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
        ESP_LOGW(TAG, "status LED encoder failed: %s", esp_err_to_name(err));
        return;
    }
    err = rmt_enable(s_channel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "status LED enable failed: %s", esp_err_to_name(err));
        return;
    }

    s_ready = true;
    status_led_set(0, 0, 16);
    ESP_LOGI(TAG, "onboard WS2812 on GPIO %d", BOARD_STATUS_LED_GPIO);
}

void status_led_set(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!s_ready) {
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
    esp_err_t err = rmt_transmit(s_channel, s_encoder, grb, sizeof(grb), &tx);
    if (err == ESP_OK) {
        err = rmt_tx_wait_all_done(s_channel, 20);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "status LED write failed: %s", esp_err_to_name(err));
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
