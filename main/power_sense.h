#pragma once

#include <stdbool.h>

void power_sense_init(void);
void power_sense_poll(void);
int power_sense_mv(void);
bool power_sense_low(void);
