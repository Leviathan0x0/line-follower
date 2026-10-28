#pragma once

#include "line_math.h"

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
