#pragma once

#include "line_math.h"
#include "line_sensors.h"

#include "esp_err.h"

bool calibration_store_load(sensor_cal_t *cal);
esp_err_t calibration_store_save(const sensor_cal_t *cal);

/* Only the operator tunables are stored. Timing limits stay with the firmware build. */
bool calibration_store_load_gains(tracker_cfg_t *cfg);
esp_err_t calibration_store_save_gains(const tracker_cfg_t *cfg);
esp_err_t calibration_store_erase_gains(void);
