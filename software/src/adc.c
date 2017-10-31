#include "adc.h"
#include "abort.h"
#include "types.h"
#include "spi.h"
#include "system.h"
#include "time_util.h"

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
     * Issue an ADC reset to place all registers into default state.
     */
    AbortIfNot(write_verify_adc_register(spi, 0, 0x80, 0x00, ms_to_ticks(10)),
            fail);

    /*
     * Configure the ADC to utilize 1.75mA drive, 16-bit 2-lane serialization,
     * and internal termination.
     */
    AbortIfNot(write_verify_adc_register(spi, 2, 0xF0, 0xF0, ms_to_ticks(10)),
            fail);

    return success;
}

result_t write_verify_adc_register(spi_driver_t *spi,
                                   const uint8_t reg,
                                   uint8_t data,
                                   uint8_t expected_response,
                                   tick_t duration)
{
    uint8_t response;
    const tick_t end = get_system_time() + duration;
    do
    {
        AbortIfNot(write_adc_register(spi, reg, data), fail);
        AbortIfNot(read_adc_register(spi, reg, &response), fail);
    }
    while (response != expected_response && get_system_time() < end);

    AbortIfNot(response == data, fail);

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
