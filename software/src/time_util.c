#include "time_util.h"

#include "system.h"

tick_t ms_to_ticks(uint32_t milliseconds)
{
    return (CPU_CLOCK_HZ / 1000 * milliseconds);
}

uint32_t ticks_to_ms(tick_t ticks)
{
    return 1000 * ticks_to_seconds(ticks);
}

tick_t micros_to_ticks(uint32_t microseconds)
{
    return (CPU_CLOCK_HZ / 1000000 * microseconds);
}

float ticks_to_seconds(tick_t ticks)
{
    return ((float)ticks) / CPU_CLOCK_HZ;
}

void busywait(tick_t wait)
{
    tick_t end = get_system_time() + wait;
    while (get_system_time() < end);
}
