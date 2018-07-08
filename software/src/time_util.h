#ifndef TIME_UTIL_H
#define TIME_UTIL_H

#include "system_params.h"
#include "types.h"

#include <sys/time.h>

int _gettimeofday( struct timeval *tv, void *tzvp);

tick_t ms_to_ticks(uint32_t milliseconds);

tick_t micros_to_ticks(uint32_t microseconds);

uint64_t ticks_to_ns(tick_t ticks);

uint32_t ticks_to_us(tick_t ticks);

uint32_t ticks_to_ms(tick_t ticks);

float ticks_to_seconds(tick_t ticks);

void busywait(tick_t wait);

#endif
