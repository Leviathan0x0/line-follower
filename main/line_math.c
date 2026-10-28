#include "line_math.h"

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

/* Straight: full cruise. Gentle curve: bleed speed. Sharp curve: crawl, so the inner wheel can reverse. */
static float forward_keep(float abs_error, float slowdown)
{
    abs_error = clampf(abs_error, 0.0f, 1.0f);
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
    float out_left = clampf(raw_left, -cfg->reverse_duty, cfg->max_duty);
    float out_right = clampf(raw_right, -cfg->reverse_duty, cfg->max_duty);

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
    return clampf(error, -1.0f, 1.0f);
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
            tr->integral = clampf(tr->integral, -cfg->i_limit, cfg->i_limit);
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
