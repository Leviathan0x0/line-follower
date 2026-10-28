#pragma once

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
