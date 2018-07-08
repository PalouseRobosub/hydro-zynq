#include "time_util.h"

#include "system.h"
#include <sys/time.h>

/**
 * Replacement function for system gettimeofday. Required by FFTW.
 *
 * @param[out] tv The location to store the current time.
 * @param[in] tzvp Unused.
 *
 * @return Zero.
 */
int _gettimeofday(struct timeval *tv, void *tzvp)
{
    uint64_t t = ticks_to_ns(get_system_time());
    tv->tv_sec = t / 1000000000;
    tv->tv_usec = ( t % 1000000000 ) / 1000;
    return 0;
}

/**
 * Converts a number of milliseconds to system timer ticks.
 *
 * @param milliseconds The number of milliseconds.
 *
 * @return The number of ticks in the provided amount.
 */
tick_t ms_to_ticks(uint32_t milliseconds)
{
    return (CPU_CLOCK_HZ / 1000 * milliseconds);
}

/**
 * Converts a number of system timer ticks to milliseconds.
 *
 * @param ticks The number of system timer ticks.
 *
 * @return The number of milliseconds in the provided tick count.
 */
uint32_t ticks_to_ms(tick_t ticks)
{
    return 1000 * ticks_to_seconds(ticks);
}

/**
 * Converts a number of system timer ticks to microseconds.
 *
 * @param ticks The number of system timer ticks.
 *
 * @return The number of microseconds in the provided tick count.
 */
uint32_t ticks_to_us(tick_t ticks)
{
    return 1000000 * ticks_to_seconds(ticks);
}

/**
 * Converts a number of system timer ticks to nanoseconds.
 *
 * @param ticks The number of system timer ticks.
 *
 * @return The number of nanoseconds in the provided tick count.
 */
uint64_t ticks_to_ns(tick_t ticks)
{
    return ((uint64_t)(1000000000.0 * ticks_to_seconds(ticks)));
}

/**
 * Converts a number of microseconds to system timer ticks.
 *
 * @param microseconds The number of microseconds.
 *
 * @return The number of ticks in the provided amount.
 */
tick_t micros_to_ticks(uint32_t microseconds)
{
    return (CPU_CLOCK_HZ / 1000000 * microseconds);
}

/**
 * Converts a number of system timer ticks to seconds.
 *
 * @param ticks The number of system timer ticks.
 *
 * @return The number of seconds in the provided tick count.
 */
float ticks_to_seconds(tick_t ticks)
{
    return ((float)ticks) / CPU_CLOCK_HZ;
}

/**
 * Waits a specified number of ticks.
 *
 * @param wait The number of ticks to wait.
 *
 * @return None.
 */
void busywait(tick_t wait)
{
    tick_t end = get_system_time() + wait;
    while (get_system_time() < end);
}
