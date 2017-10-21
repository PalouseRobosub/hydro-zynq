#include "time_util.h"

tick_t ms_to_ticks(uint32_t milliseconds)
{
    return (CPU_CLOCK_HZ / 1000 * milliseconds);
}
