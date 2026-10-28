#include "follower.h"

#include "board_config.h"
#include "button.h"
#include "calibrate.h"
#include "calibration_store.h"
#include "line_sensors.h"
#include "motors.h"
#include "power_sense.h"
#include "status_led.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "follow";

static tracker_t s_tr;
static tracker_cfg_t s_cfg;
static follower_snapshot_t s_snap;
static SemaphoreHandle_t s_mu;
static QueueHandle_t s_cmds;
static bool s_logging = true;
static bool s_low_latched;
static int64_t s_last_us;
static TickType_t s_wake;

static float clampf(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static void format_fixed(char *dst, size_t len, float value)
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
    format_fixed(err_text, sizeof(err_text), dt_error);
    ESP_LOGI(TAG, "%s pos=%d err=%s cmd=%d,%d blk=%d %d %d %d %d",
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
        ESP_LOGE(TAG, "calibration was not stored: %s", esp_err_to_name(err));
    }
    tracker_init(&s_tr);
    button_poll_reset();
    arm_clock();
    ESP_LOGI(TAG, "Paused. Short-press BOOT or type `go` when the robot is on the line.");
}

static bool resume_motion(void)
{
    if (s_low_latched && power_sense_low()) {
        ESP_LOGW(TAG, "Pack is still under %d mV (%d mV). Charge it before driving.",
                 BOARD_VBAT_CUTOFF_MV, power_sense_mv());
        return false;
    }
    s_low_latched = false;
    if (s_tr.paused || s_tr.state == LINE_TRACK_STOPPED || s_tr.state == LINE_TRACK_PAUSED) {
        tracker_set_paused(&s_tr, false);
        ESP_LOGI(TAG, "Running");
    }
    return true;
}

static void pause_motion(void)
{
    if (!s_tr.paused && s_tr.state != LINE_TRACK_STOPPED) {
        tracker_set_paused(&s_tr, true);
        motors_brake();
        ESP_LOGI(TAG, "Paused");
    }
}

static void nudge_motors(void)
{
    bool resume = s_tr.state == LINE_TRACK_FOLLOW || s_tr.state == LINE_TRACK_GRACE ||
                  s_tr.state == LINE_TRACK_SEARCH;
    tracker_set_paused(&s_tr, true);
    motors_brake();
    vTaskDelay(pdMS_TO_TICKS(30));
    ESP_LOGI(TAG, "Nudging both wheels forward for 400 ms. Hold the chassis up.");
    motors_set_signed(360, 360);
    vTaskDelay(pdMS_TO_TICKS(400));
    motors_brake();
    ESP_LOGI(TAG, "Each wheel should have rolled forward. If one ran backward, set that BOARD_MOTOR_*_POLARITY to -1 or swap its two wires.");
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
        ESP_LOGI(TAG, "live log %s", cmd == FOLLOW_CMD_LOG_ON ? "on" : "off");
        break;
    default:
        break;
    }
}

static void offer_recal_window(void)
{
    ESP_LOGI(TAG, "Stored calibration found. Press BOOT within %d ms to redo it.", BOARD_RECAL_WINDOW_MS);
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
        ESP_LOGI(TAG, "Calibration loaded");
        offer_recal_window();
    } else {
        ESP_LOGW(TAG, "No calibration stored. Starting the white/black capture.");
        run_calibration();
    }

    tracker_init(&s_tr);
    button_poll_reset();
    arm_clock();
    ESP_LOGI(TAG, "Paused with the brake on. Short-press BOOT or type `go` to follow. `help` lists commands.");

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
                ESP_LOGW(TAG, "Pack %d mV is under %d mV. Motors held.",
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
        ESP_LOGI(TAG, "using built-in gains");
    }
    tracker_init(&s_tr);
    s_mu = xSemaphoreCreateMutex();
    s_cmds = xQueueCreate(8, sizeof(follow_cmd_t));
    if (s_mu == NULL || s_cmds == NULL) {
        ESP_LOGE(TAG, "no memory for the follower task");
        return;
    }
    s_snap.state = LINE_TRACK_PAUSED;
    s_snap.logging = true;
    s_snap.vbat_mv = -1;
    if (xTaskCreate(control_task, "follow", 8192, NULL, 8, NULL) != pdPASS) {
        ESP_LOGE(TAG, "follower task was not created");
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
    float clamped = clampf(value, lo, hi);
    lock();
    *field = clamped;
    unlock();
    char text[16];
    format_fixed(text, sizeof(text), clamped);
    ESP_LOGI(TAG, "%s = %s", name, text);
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
        ESP_LOGE(TAG, "could not save gains: %s", esp_err_to_name(err));
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
        ESP_LOGE(TAG, "could not erase gains: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "gains restored to the built-in defaults");
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
