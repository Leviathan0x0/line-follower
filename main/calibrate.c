#include "calibrate.h"

#include "board_config.h"
#include "button.h"
#include "calibration_store.h"
#include "line_sensors.h"
#include "status_led.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "cal";

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
    ESP_LOGI(TAG, "Hold BOOT and rock the bar across the tape until every sensor has seen black. Release BOOT to finish.");
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
        ESP_LOGW(TAG, "That hold was %d ms. Keep BOOT down while you rock across the line.", held_ms);
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
        ESP_LOGI(TAG, "GPIO %d (%s) white=%d black=%d span=%d %s raw %d..%d",
                 line_sensors_gpio(i), k_side[logical_index(i)], cal->white[i], black, span, sense,
                 min_raw[i], max_raw[i]);
        if (span < BOARD_CAL_MIN_SPAN) {
            ESP_LOGW(TAG, "GPIO %d did not see enough contrast. Rock that sensor over the tape.",
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

    ESP_LOGI(TAG, "Step 1/2: set all five sensors on bare white track, then press BOOT.");
    wait_for_press(0, 0, 36);
    wait_for_release();
    sample_white(&cal);
    ESP_LOGI(TAG, "White raw: %d %d %d %d %d",
             cal.white[0], cal.white[1], cal.white[2], cal.white[3], cal.white[4]);

    for (;;) {
        ESP_LOGI(TAG, "Step 2/2: black tape. Index 0 in the log is the sensor on GPIO %d.",
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
