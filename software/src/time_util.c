#include "time_util.h"

tick_t ms_to_ticks(uint32_t milliseconds)
{
    return (CPU_CLOCK_HZ / 1000 * milliseconds);
}

uint32_t ticks_to_ms(tick_t ticks)
{
    return 1000 * ticks_to_seconds(ticks);
}

float ticks_to_seconds(tick_t ticks)
{
    return ((float)ticks) / CPU_CLOCK_HZ;
}
