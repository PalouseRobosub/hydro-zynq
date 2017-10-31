#include "xadc.h"
#include "types.h"
#include "abort.h"
#include "uart.h"

result_t init_xadc(xadc_t *xadc, uint32_t base_address)
{
    AbortIfNot(xadc, fail);
    AbortIfNot(base_address, fail);

    xadc->regs = (struct XadcRegs *)base_address;

    /*
     * Trigger a reset of the XADC.
     */
    xadc->regs->SRR = 0xA;

    /*
     * Wait for a sample to complete.
     */
    while((xadc->regs->SR & (1 << 5)) == 0);

    return success;
}

result_t read_xadc_temperature(xadc_t *xadc, float *temp_c)
{
    AbortIfNot(xadc, fail);
    AbortIfNot(temp_c, fail);

    AbortIfNot(xadc->regs, fail);

    /*
     * Check to ensure a new temperature reading is ready.
     */
    AbortIfNot(xadc->regs->SR & (1 << 5), fail);

    /*
     * Read the temperature measurement.
     */
    uint32_t reading = xadc->regs->TEMP;

    /*
     * Convert the raw reading to celcius (taken from XSysMon code examples).
     */
    *temp_c = (reading / 65536.0) / 0.00198421639 - 273.15;

    return success;
}
