#pragma once

#include <stdbool.h>

typedef enum {
    BUTTON_NONE = 0,
    BUTTON_SHORT,
    BUTTON_LONG,
} button_event_t;

void button_init(void);
bool button_is_down(void);
button_event_t button_poll(int dt_ms);
void button_poll_reset(void);
