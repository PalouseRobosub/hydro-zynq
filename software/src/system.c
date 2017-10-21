#include "system.h"

#include "abort.h"
#include "global_timer.h"
//#include "ps7_init.h"
#include "types.h"
#include "regs/gpio_regs.h"

/**
 * Initializes the processing system.
 *
 * @return Success or fail.
 */
result_t init_system()
{
    /*
     * Initialize the processing system.
     */
    //AbortIfNot(ps7_init() == PS7_INIT_SUCCESS, fail);

    /*
     * Initialize the user indication LED.
     */
    gpio_regs->OEN_1 |= 1 << (47 - 32);
    gpio_regs->DIRM_1  |= 1 << (47 - 32);
    gpio_regs->DATA[1] = 0;

    /*
     * Initialize the global system timer.
     */
    AbortIfNot(init_global_timer(), fail);

    return success;
}

void set_board_led(bool enabled)
{
    uint32_t data = gpio_regs->DATA[1];
    gpio_regs->DATA[1] = ((enabled)? data | (1 << 47 - 32) : data & ~(1 << 47 - 32));
}

/**
 * Gets the current system time in clock ticks.
 *
 * @return The number of ticks since the system first started.
 */
tick_t get_system_time()
{
    /*
     * Read the global timer value.
     */
    return get_global_timer_count();
}
