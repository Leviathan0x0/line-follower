#pragma once

#include "esp_err.h"

/* Signed command is -BOARD_DUTY_MAX … BOARD_DUTY_MAX. Positive is forward. 0 brakes. */
esp_err_t motors_init(void);
void motors_set_signed(int left, int right);
void motors_brake(void);
