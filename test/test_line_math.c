#include "line_math.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int g_failures = 0;

#define EXPECT(cond)                                                          \
    do {                                                                      \
        if (!(cond)) {                                                        \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

static tracker_cfg_t test_cfg(void)
{
    tracker_cfg_t cfg = {
        .base_duty = 480.0f,
        .max_duty = 1024.0f,
        .reverse_duty = 280.0f,
        .kp = 780.0f,
        .kd = 12.0f,
        .ki = 40.0f,
        .slowdown = 0.55f,
        .i_limit = 0.40f,
        .search_duty = 400.0f,
        .deriv_filter = 0.35f,
        .grace_ms = 160,
        .timeout_ms = 1400,
    };
    return cfg;
}

static void test_normalize(void)
{
    EXPECT(line_normalize(1000, 1000, 3000) == 0);
    EXPECT(line_normalize(3000, 1000, 3000) == 1000);
    EXPECT(line_normalize(2000, 1000, 3000) == 500);
    EXPECT(line_normalize(0, 1000, 3000) == 0);
    EXPECT(line_normalize(5000, 1000, 3000) == 1000);

    EXPECT(line_normalize(3000, 3000, 1000) == 0);
    EXPECT(line_normalize(1000, 3000, 1000) == 1000);
    EXPECT(line_normalize(2000, 3000, 1000) == 500);
    EXPECT(line_normalize(4000, 3000, 1000) == 0);
    EXPECT(line_normalize(0, 3000, 1000) == 1000);

    EXPECT(line_normalize(1234, 800, 800) == 0);
}

static void test_centroid(void)
{
    int position = -1;
    int total = -1;
    const int center_only[LINE_SENSOR_COUNT] = {40, 30, 900, 20, 50};
    EXPECT(line_centroid(center_only, 150, &position, &total));
    EXPECT(position == LINE_POS_CENTER);
    EXPECT(total == 900);

    const int far_right[LINE_SENSOR_COUNT] = {0, 0, 0, 0, 1000};
    EXPECT(line_centroid(far_right, 150, &position, &total));
    EXPECT(position == LINE_POS_MAX);

    const int far_left[LINE_SENSOR_COUNT] = {1000, 0, 0, 0, 0};
    EXPECT(line_centroid(far_left, 150, &position, &total));
    EXPECT(position == 0);

    const int left_pair[LINE_SENSOR_COUNT] = {1000, 1000, 0, 0, 0};
    EXPECT(line_centroid(left_pair, 150, &position, &total));
    EXPECT(position == 500);

    const int all_black[LINE_SENSOR_COUNT] = {1000, 1000, 1000, 1000, 1000};
    EXPECT(line_centroid(all_black, 150, &position, &total));
    EXPECT(position == LINE_POS_CENTER);
    EXPECT(total == 5000);

    const int paper[LINE_SENSOR_COUNT] = {40, 30, 20, 20, 50};
    EXPECT(!line_centroid(paper, 150, &position, &total));
    EXPECT(position == LINE_POS_CENTER);
    EXPECT(total == 0);

    const int at_threshold[LINE_SENSOR_COUNT] = {0, 0, 150, 0, 0};
    EXPECT(line_centroid(at_threshold, 150, &position, &total));
    EXPECT(position == LINE_POS_CENTER);
}

static void test_mix_and_tracker(void)
{
    tracker_cfg_t cfg = test_cfg();
    tracker_t tr;
    tracker_init(&tr);
    tracker_set_paused(&tr, false);

    tracker_step(&tr, &cfg, true, LINE_POS_CENTER, 0.01f);
    EXPECT(tr.state == LINE_TRACK_FOLLOW);
    EXPECT(fabsf(tr.left - 480.0f) < 0.01f);
    EXPECT(fabsf(tr.right - 480.0f) < 0.01f);

    tracker_init(&tr);
    tracker_set_paused(&tr, false);
    tracker_step(&tr, &cfg, true, 2500, 0.01f);
    EXPECT(tr.left > tr.right);
    EXPECT(tr.left > 0.0f);
    EXPECT(tr.right > 0.0f);
    float average = 0.5f * (tr.left + tr.right);
    EXPECT(average < 470.0f);
    EXPECT(average > 390.0f);

    tracker_init(&tr);
    tracker_set_paused(&tr, false);
    tracker_step(&tr, &cfg, true, LINE_POS_MAX, 0.01f);
    EXPECT(tr.left > 800.0f);
    EXPECT(fabsf(tr.right - (-280.0f)) < 0.01f);

    tracker_init(&tr);
    tracker_set_paused(&tr, false);
    tracker_step(&tr, &cfg, true, 0, 0.01f);
    EXPECT(fabsf(tr.left - (-280.0f)) < 0.01f);
    EXPECT(tr.right > 800.0f);

    float left_still = 0.0f;
    float right_still = 0.0f;
    float left_rising = 0.0f;
    float right_rising = 0.0f;
    line_mix_duties(&cfg, 0.3f, 0.0f, 0.0f, &left_still, &right_still);
    line_mix_duties(&cfg, 0.3f, 5.0f, 0.0f, &left_rising, &right_rising);
    EXPECT(left_rising > left_still + 50.0f);
    EXPECT(right_rising < right_still - 50.0f);

    tracker_init(&tr);
    tracker_set_paused(&tr, false);
    cfg.kp = 10.0f;
    for (int i = 0; i < 100; i++) {
        tracker_step(&tr, &cfg, true, 2400, 0.01f);
    }
    EXPECT(tr.integral > 0.15f);
    EXPECT(tr.integral < 0.25f);

    tracker_init(&tr);
    tracker_set_paused(&tr, false);
    cfg = test_cfg();
    cfg.kp = 5000.0f;
    for (int i = 0; i < 50; i++) {
        tracker_step(&tr, &cfg, true, 2800, 0.01f);
    }
    EXPECT(fabsf(tr.integral) < 0.02f);
    EXPECT(tr.left <= cfg.max_duty + 0.01f);
    EXPECT(tr.right >= -cfg.reverse_duty - 0.01f);
}

static void test_lost_line(void)
{
    tracker_cfg_t cfg = test_cfg();
    tracker_t tr;
    tracker_init(&tr);
    tracker_set_paused(&tr, false);

    tracker_step(&tr, &cfg, true, 3000, 0.01f);
    float hold_left = tr.left;
    float hold_right = tr.right;
    EXPECT(hold_left > hold_right);
    EXPECT(tr.last_sign > 0);

    for (int i = 0; i < 16; i++) {
        tracker_step(&tr, &cfg, false, 0, 0.01f);
        EXPECT(tr.state == LINE_TRACK_GRACE);
        EXPECT(fabsf(tr.left - hold_left) < 0.01f);
        EXPECT(fabsf(tr.right - hold_right) < 0.01f);
    }

    tracker_step(&tr, &cfg, false, 0, 0.01f);
    EXPECT(tr.state == LINE_TRACK_SEARCH);
    EXPECT(fabsf(tr.left - 400.0f) < 0.01f);
    EXPECT(fabsf(tr.right - (-400.0f)) < 0.01f);

    tracker_init(&tr);
    tracker_set_paused(&tr, false);
    tracker_step(&tr, &cfg, true, 1000, 0.01f);
    EXPECT(tr.last_sign < 0);
    for (int i = 0; i < 17; i++) {
        tracker_step(&tr, &cfg, false, 0, 0.01f);
    }
    EXPECT(tr.state == LINE_TRACK_SEARCH);
    EXPECT(fabsf(tr.left - (-400.0f)) < 0.01f);
    EXPECT(fabsf(tr.right - 400.0f) < 0.01f);

    int guard = 0;
    while (tr.state != LINE_TRACK_STOPPED && guard < 400) {
        tracker_step(&tr, &cfg, false, 0, 0.01f);
        guard++;
    }
    EXPECT(tr.state == LINE_TRACK_STOPPED);
    EXPECT(tr.left == 0.0f);
    EXPECT(tr.right == 0.0f);
    EXPECT(guard < 400);

    tracker_step(&tr, &cfg, true, LINE_POS_CENTER, 0.01f);
    EXPECT(tr.state == LINE_TRACK_STOPPED);
    EXPECT(tr.left == 0.0f);
    EXPECT(tr.right == 0.0f);

    tracker_set_paused(&tr, false);
    tracker_step(&tr, &cfg, true, LINE_POS_CENTER, 0.01f);
    EXPECT(tr.state == LINE_TRACK_FOLLOW);
    EXPECT(tr.left > 400.0f);
    EXPECT(tr.right > 400.0f);

    tracker_set_paused(&tr, true);
    tracker_step(&tr, &cfg, true, LINE_POS_MAX, 0.01f);
    EXPECT(tr.state == LINE_TRACK_PAUSED);
    EXPECT(tr.left == 0.0f);
    EXPECT(tr.right == 0.0f);
}

int main(void)
{
    test_normalize();
    test_centroid();
    test_mix_and_tracker();
    test_lost_line();

    if (g_failures != 0) {
        fprintf(stderr, "%d assertion(s) failed\n", g_failures);
        return 1;
    }
    printf("line_math: all tests passed\n");
    return 0;
}
