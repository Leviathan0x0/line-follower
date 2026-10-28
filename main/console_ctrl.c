#include "console_ctrl.h"

#include "board_config.h"
#include "follower.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "cli";

static void format_fixed(char *dst, size_t len, float value)
{
    int milli = (int)(value >= 0.0f ? value * 1000.0f + 0.5f : value * 1000.0f - 0.5f);
    int mag = milli < 0 ? -milli : milli;
    snprintf(dst, len, "%s%d.%03d", milli < 0 ? "-" : "", mag / 1000, mag % 1000);
}

static void print_help(void)
{
    ESP_LOGI(TAG, "help                  this list");
    ESP_LOGI(TAG, "show                  sensors, error, wheel commands");
    ESP_LOGI(TAG, "pins                  wiring map");
    ESP_LOGI(TAG, "bat                   pack voltage, if sensing is enabled");
    ESP_LOGI(TAG, "go / pause / stop     start or brake");
    ESP_LOGI(TAG, "nudge                 roll both wheels forward for 0.4 s");
    ESP_LOGI(TAG, "cal                   white/black capture");
    ESP_LOGI(TAG, "log on|off            2 Hz status line");
    ESP_LOGI(TAG, "kp|kd|ki|speed|slow   print or set a gain");
    ESP_LOGI(TAG, "save                  store kp kd ki speed slow");
    ESP_LOGI(TAG, "defaults              restore those five built-in values");
}

static void print_pins(void)
{
    ESP_LOGI(TAG, "sensors L->R GPIO %d %d %d %d %d (ADC, 3.3 V)",
             BOARD_SENSOR0_GPIO, BOARD_SENSOR1_GPIO, BOARD_SENSOR2_GPIO,
             BOARD_SENSOR3_GPIO, BOARD_SENSOR4_GPIO);
    ESP_LOGI(TAG, "left  IN1=%d IN2=%d ENA=%d", BOARD_LEFT_IN1_GPIO, BOARD_LEFT_IN2_GPIO, BOARD_LEFT_EN_GPIO);
    ESP_LOGI(TAG, "right IN3=%d IN4=%d ENB=%d", BOARD_RIGHT_IN1_GPIO, BOARD_RIGHT_IN2_GPIO, BOARD_RIGHT_EN_GPIO);
    ESP_LOGI(TAG, "BOOT button GPIO %d, WS2812 GPIO %d", BOARD_BUTTON_GPIO, BOARD_STATUS_LED_GPIO);
    ESP_LOGI(TAG, "sensor reverse=%d  left polarity=%d  right polarity=%d",
             BOARD_SENSOR_REVERSE, BOARD_MOTOR_LEFT_POLARITY, BOARD_MOTOR_RIGHT_POLARITY);
}

static void print_show(void)
{
    follower_snapshot_t snap;
    if (!follower_copy_snapshot(&snap)) {
        ESP_LOGW(TAG, "follower is not running");
        return;
    }
    char err_text[16];
    char kp[16];
    char kd[16];
    char ki[16];
    char speed[16];
    char slow[16];
    format_fixed(err_text, sizeof(err_text), snap.error);
    format_fixed(kp, sizeof(kp), snap.kp);
    format_fixed(kd, sizeof(kd), snap.kd);
    format_fixed(ki, sizeof(ki), snap.ki);
    format_fixed(speed, sizeof(speed), snap.base_duty);
    format_fixed(slow, sizeof(slow), snap.slowdown);
    ESP_LOGI(TAG, "%s pos=%d err=%s seen=%d low=%d bat=%d",
             line_track_state_name(snap.state), snap.position, err_text,
             snap.seen ? 1 : 0, snap.low_battery ? 1 : 0, snap.vbat_mv);
    ESP_LOGI(TAG, "cmd L=%d R=%d  blk=%d %d %d %d %d",
             (int)(snap.left >= 0 ? snap.left + 0.5f : snap.left - 0.5f),
             (int)(snap.right >= 0 ? snap.right + 0.5f : snap.right - 0.5f),
             snap.value[0], snap.value[1], snap.value[2], snap.value[3], snap.value[4]);
    ESP_LOGI(TAG, "raw L->R %d %d %d %d %d",
             snap.raw[0], snap.raw[1], snap.raw[2], snap.raw[3], snap.raw[4]);
    ESP_LOGI(TAG, "speed=%s kp=%s kd=%s ki=%s slow=%s log=%s",
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
                ESP_LOGI(TAG, "pack sensing is compiled out");
            } else {
                ESP_LOGI(TAG, "pack %d mV", snap.vbat_mv);
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
        ESP_LOGW(TAG, "unknown command '%s'", line);
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
