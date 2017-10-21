#ifndef TIME_UTIL_H
#define TIME_UTIL_H

#include "system_params.h"
#include "types.h"

tick_t ms_to_ticks(uint32_t milliseconds);

uint32_t ticks_to_ms(tick_t ticks);

float ticks_to_seconds(tick_t ticks);

#endif
