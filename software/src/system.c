#include "system.h"

#include "abort.h"
#include "global_timer.h"
#include "xscugic.h"
#include "types.h"
#include "regs/gpio_regs.h"
#include "regs/slcr_regs.h"

/**
 * Initializes the processing system.
 *
 * @note The processing system is initialized by the FSBL.
 *
 * @return Success or fail.
 */
result_t init_system()
{
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

    /*
     * Set up Xilinx's interrupt controller driver.
     */
    set_interrupts(false);
    XScuGic_DeviceInitialize(XPAR_SCUGIC_SINGLE_DEVICE_ID);

    /*
     * Register the GIC handler with the zynq trampoline.
     */
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
            (Xil_ExceptionHandler)XScuGic_DeviceInterruptHandler,
            (void *)XPAR_SCUGIC_SINGLE_DEVICE_ID);

    return success;
}

/**
 * Fatal exit conditional. Reset the processor.
 *
 * @return None.
 */
void give_up()
{
    /*
     * Unlock SLC registers.
     */
    SLCR->SLCR_UNLOCK = 0xDF0D;

    /*
     * Perform a reset of the processing system (PS).
     */
    SLCR->PSS_RST_CTRL = 1;
}

/**
 * Enable or disable interrupts.
 *
 * @bool enabled True if interrupts should be enabled.
 *
 * @return None.
 */
void set_interrupts(bool enabled)
{
    if (enabled)
    {
        __asm volatile("push {r1}\n"
                       "mrs r1, cpsr\n"
                       "bic r1, r1, #0x80\n"
                       "msr cpsr, r1\n"
                       "pop {r1}\n");
    }
    else
    {
        __asm volatile("push {r1}\n"
                       "mrs r1, cpsr\n"
                       "orr r1, r1, #0x80\n"
                       "msr cpsr, r1\n"
                       "pop {r1}\n");
    }
}

/**
 * Sets the status of the on-board LED for the MicroZed.
 *
 * @param enabled True if the LED should be turned on.
 *
 * @return None.
 */
void set_board_led(bool enabled)
{
    uint32_t data = gpio_regs->DATA[1];
    gpio_regs->DATA[1] = ((enabled)? data | (1 << (47 - 32)) : data & ~(1 << (47 - 32)));
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
