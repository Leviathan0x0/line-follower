#pragma once

#include <stdint.h>

void status_led_init(void);
void status_led_set(uint8_t red, uint8_t green, uint8_t blue);
