#pragma once

#include "esp_err.h"

/* Blocks until every sensor has a usable black/white span. Applies and stores it. */
esp_err_t calibrate_interactive(void);
