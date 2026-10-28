#pragma once

#include "line_math.h"

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
