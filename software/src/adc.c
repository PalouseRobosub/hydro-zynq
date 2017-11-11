#include "adc.h"
#include "abort.h"
#include "types.h"
#include "spi.h"
#include "system.h"
#include "time_util.h"

result_t init_adc(spi_driver_t *spi, int verify)
{
    AbortIfNot(spi, fail);
    AbortIfNot(spi->regs, fail);

    /*
     * Reset the ADC using a software reset.
     */
    if (verify)
    {
        AbortIfNot(write_verify_adc_register(spi, 0, 0x80, 0x00), fail);

        /*
         * Set up the test pattern.
         */
        AbortIfNot(write_verify_adc_register(spi, 4, 0x0F, 0x0F), fail);
        //AbortIfNot(write_verify_adc_register(spi, 3, 0x80, 0x80), fail);
    }
    else
    {
        AbortIfNot(write_adc_register(spi, 0, 0x80), fail);

        /*
         * Set up the test pattern.
         */
        AbortIfNot(write_adc_register(spi, 4, 0x0F), fail);
        //AbortIfNot(write_adc_register(spi, 3, 0x80), fail);
    }

    return success;
}

result_t write_verify_adc_register(spi_driver_t *spi,
                                   const uint8_t reg,
                                   uint8_t data,
                                   uint8_t expected_response)
{
    uint8_t response;

    AbortIfNot(write_adc_register(spi, reg, data), fail);
    AbortIfNot(read_adc_register(spi, reg, &response), fail);

    AbortIfNot(response == expected_response, fail);

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
