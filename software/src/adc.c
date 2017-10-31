#include "adc.h"
#include "abort.h"
#include "types.h"
#include "spi.h"

result_t init_adc(spi_driver_t *spi)
{
    AbortIfNot(spi, fail);
    AbortIfNot(spi->regs, fail);

    /*
     * Reset the ADC using a software reset.
     */
    AbortIfNot(write_adc_register(spi, 0, 0x80), fail);

    /*
     * The ADC should exit reset immediately.
     */
    uint8_t reg;
    AbortIfNot(read_adc_register(spi, 0, &reg), fail);
    AbortIfNot(reg == 0, fail);

    /*
     * Write to the test pattern LSB register as a scratchpad to verify we can
     * talk to the ADC.
     */
    AbortIfNot(write_adc_register(spi, 4, 0xab), fail);
    AbortIfNot(read_adc_register(spi, 4, &reg), fail);
    AbortIfNot(reg == 0xab, fail);

    return success;
}

result_t write_adc_register(spi_driver_t *spi, const uint8_t reg, uint8_t data)
{
    AbortIfNot(reg <= 4, fail);

    uint16_t command = ((uint16_t)reg) << 8 | data;
    uint16_t response;
    AbortIfNot(transact_spi(spi, command, &response), fail);

    return success;
}

result_t read_adc_register(spi_driver_t *spi, const uint8_t reg, uint8_t *data)
{
    AbortIfNot(reg <= 4, fail);
    AbortIfNot(data, fail);

    uint16_t command =  1 << 15 | ((uint16_t)reg) << 8;
    uint16_t response;
    AbortIfNot(transact_spi(spi, command, &response), fail);

    *data = response & 0xFF;

    return success;
}
