#include "global_timer.h"
#include "regs/system_registers.h"

#include "abort.h"

result_t init_global_timer()
{
    AbortIfNot(global_timer_regs, fail);

    /*
     * Disable the global timer.
     */
    global_timer_regs->Control_Register = 0;

    /*
     * Set the timer values to zero.
     */
    global_timer_regs->Counter_Register[0] = 0;
    global_timer_regs->Counter_Register[1] = 0;

    /*
     * Enable the global timer without a prescaler.
     */
    global_timer_regs->Control_Register = 1;

    return success;
}

uint64_t get_global_timer_count()
{
    /*
     * Read the upper portion.
     */
    uint32_t upper_portion = global_timer_regs->Counter_Register[1];
    uint32_t lower_portion = global_timer_regs->Counter_Register[0];

    /*
     * Verify that rollover didn't occur between reading the high register and
     * reading the low register.
     */
    uint32_t double_read = global_timer_regs->Counter_Register[1];
    while (double_read != upper_portion)
    {
        upper_portion = double_read;
        lower_portion = global_timer_regs->Counter_Register[0];
        double_read = global_timer_regs->Counter_Register[1];
    }

    /*
     * Convert the two 32 bit portions into a single 64 bit result.
     */
    return ((((uint64_t)upper_portion) << 32) | lower_portion);
}
