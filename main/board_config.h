#pragma once

#include "driver/gpio.h"

/*
 * ESP32-S3 DevKit line follower.
 *
 * Sensor 0 is the far-left channel when you face the direction of travel.
 * The array belongs on the front lip of the chassis, 8–12 mm above the track.
 *
 * Assumed motors are 6 V N20 gearmotors. The L298N drops about 2 V, so the
 * motor supply is a 2S pack (see README), not the 5 V logic rail.
 */

#define BOARD_SENSOR_COUNT 5

#define BOARD_SENSOR0_GPIO GPIO_NUM_1
#define BOARD_SENSOR1_GPIO GPIO_NUM_2
#define BOARD_SENSOR2_GPIO GPIO_NUM_4
#define BOARD_SENSOR3_GPIO GPIO_NUM_5
#define BOARD_SENSOR4_GPIO GPIO_NUM_6

#define BOARD_LEFT_IN1_GPIO GPIO_NUM_7
#define BOARD_LEFT_IN2_GPIO GPIO_NUM_8
#define BOARD_LEFT_EN_GPIO  GPIO_NUM_9

#define BOARD_RIGHT_IN1_GPIO GPIO_NUM_10
#define BOARD_RIGHT_IN2_GPIO GPIO_NUM_11
#define BOARD_RIGHT_EN_GPIO  GPIO_NUM_12

#define BOARD_BUTTON_GPIO GPIO_NUM_0
#define BOARD_STATUS_LED_GPIO GPIO_NUM_48

/* Set to 1 if the sensor PCB is mounted with channel 0 on the robot's right. */
#define BOARD_SENSOR_REVERSE 0

/* +1 if IN1-high / IN2-low drives that wheel forward. Use -1 if it runs backward. */
#define BOARD_MOTOR_LEFT_POLARITY  1
#define BOARD_MOTOR_RIGHT_POLARITY 1

/* L298N is a slow bipolar bridge. 1 kHz keeps it out of its switching-loss region. */
#define BOARD_PWM_FREQ_HZ 1000
#define BOARD_PWM_BITS 10
#define BOARD_DUTY_MAX (1 << BOARD_PWM_BITS)

/*
 * Command 0 brakes. Any non-zero command is mapped onto
 * [BOARD_MOTOR_START_DUTY, BOARD_DUTY_MAX] so a geared N20 actually starts
 * instead of sitting in the L298N dead zone and making heat.
 */
#define BOARD_MOTOR_START_DUTY 200
#define BOARD_MOTOR_CMD_DEADBAND 30

#define BOARD_CONTROL_PERIOD_MS 10
#define BOARD_ADC_SAMPLES 8

/* Normalized blackness (0 white … 1000 black) below this does not vote. */
#define BOARD_BLACK_THRESHOLD 180

/* Minimum |black - white| raw counts accepted for one sensor during calibration. */
#define BOARD_CAL_MIN_SPAN 300

#define BOARD_BASE_DUTY 480.0f
#define BOARD_MAX_DUTY ((float)BOARD_DUTY_MAX)
#define BOARD_REVERSE_DUTY 280.0f
#define BOARD_KP 780.0f
#define BOARD_KD 12.0f
#define BOARD_KI 40.0f
#define BOARD_SLOWDOWN 0.55f
#define BOARD_I_LIMIT 0.40f
#define BOARD_SEARCH_DUTY 400.0f
#define BOARD_DERIV_FILTER 0.35f
#define BOARD_GRACE_MS 160
#define BOARD_LOST_TIMEOUT_MS 1400

#define BOARD_BUTTON_SHORT_MS 50
#define BOARD_BUTTON_LONG_MS 1200
#define BOARD_RECAL_WINDOW_MS 3000

/*
 * Optional pack measurement. Default off so a floating GPIO cannot look like
 * a dead battery. When enabled, wire a divider from battery + to GPIO14:
 * 22k from pack to the pin, 10k from the pin to GND. 8.4 V then sits near 2.6 V.
 */
#define BOARD_BATTERY_SENSE_ENABLE 0
#define BOARD_VBAT_GPIO GPIO_NUM_14
#define BOARD_VBAT_R_TOP 22000
#define BOARD_VBAT_R_BOT 10000
#define BOARD_VBAT_CUTOFF_MV 6600
#define BOARD_VBAT_LOW_SAMPLES 5

#define BOARD_STATUS_LED_ENABLE 1
