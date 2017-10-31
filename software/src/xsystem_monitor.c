#include "xsystem_monitor.h"
#include "types.h"
#include "abort.h"
#include "uart.h"

result_t init_xsystem_monitor(xsystem_monitor_t *xsystem_monitor, uint32_t base_address)
{
    AbortIfNot(xsystem_monitor, fail);
    AbortIfNot(base_address, fail);

    xsystem_monitor->regs = (struct XSysMonRegs *)base_address;

    /*
     * Trigger a reset of the XADC.
     */
    xsystem_monitor->regs->SRR = 0xA;

    /*
     * Wait for a sample to complete.
     */
    while((xsystem_monitor->regs->SR & (1 << 5)) == 0);

    return success;
}

result_t read_fpga_temperature(xsystem_monitor_t *xsystem_monitor, float *temp_c)
{
    AbortIfNot(xsystem_monitor, fail);
    AbortIfNot(temp_c, fail);

    AbortIfNot(xsystem_monitor->regs, fail);

    /*
     * Check to ensure a new temperature reading is ready.
     */
    AbortIfNot(xsystem_monitor->regs->SR & (1 << 5), fail);

    /*
     * Read the temperature measurement.
     */
    uint32_t reading = xsystem_monitor->regs->TEMP;

    /*
     * Convert the raw reading to celcius (taken from XSysMon code examples).
     */
    *temp_c = (reading / 65536.0) / 0.00198421639 - 273.15;

    return success;
}
