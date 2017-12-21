#include "adc.h"
#include "abort.h"
#include "types.h"
#include "spi.h"
#include "system.h"
#include "time_util.h"
#include "db.h"

result_t init_adc(adc_driver_t *adc, spi_driver_t *spi, uint32_t addr, bool verify, bool test_pattern)
{
    AbortIfNot(spi, fail);
    AbortIfNot(spi->regs, fail);

    AbortIfNot(adc, fail);
    adc->spi = spi;
    adc->regs = (struct AdcRegs *)addr;

    /*
     * Reset the ADC using a software reset.
     */
    if (verify)
    {
        AbortIfNot(write_verify_adc_register(adc, 0, 0x80, 0x00), fail);
        AbortIfNot(write_verify_adc_register(adc, 3, 0x00, 0x00), fail);

        //AbortIfNot(write_verify_adc_register(adc, 1, 1 << 5, 1 << 5), fail);

        /*
         * Set up the test pattern.
         */
        if (test_pattern)
        {
            AbortIfNot(write_verify_adc_register(adc, 4, 0x0F, 0x0f), fail);
            AbortIfNot(write_verify_adc_register(adc, 3, 0x80, 0x80), fail);
        }
    }
    else
    {
        AbortIfNot(write_adc_register(adc, 0, 0x80), fail);
        AbortIfNot(write_adc_register(adc, 3, 0x00), fail);

        //AbortIfNot(write_adc_register(adc, 1, 1 << 5), fail);

        /*
         * Set up the test pattern.
         */
        if (test_pattern)
        {
            AbortIfNot(write_adc_register(adc, 4, 0xaa), fail);
            AbortIfNot(write_adc_register(adc, 3, 0x8a), fail);
        }
    }

    return success;
}

result_t write_verify_adc_register(adc_driver_t *adc,
                                   const uint8_t reg,
                                   uint8_t data,
                                   uint8_t expected_response)
{
    uint8_t response;

    AbortIfNot(write_adc_register(adc, reg, data), fail);
    AbortIfNot(read_adc_register(adc, reg, &response), fail);

    dbprintf("Register %d: %x\n", reg, (int)response);
    AbortIfNot(response == expected_response, fail);

    return success;
}

result_t write_adc_register(adc_driver_t *adc, const uint8_t reg, uint8_t data)
{
    AbortIfNot(reg <= 4, fail);

    uint16_t command = ((uint16_t)reg) << 8 | data;
    uint16_t response;
    AbortIfNot(transact_spi(adc->spi, command, &response), fail);

    return success;
}

result_t read_adc_register(adc_driver_t *adc, const uint8_t reg, uint8_t *data)
{
    AbortIfNot(reg <= 4, fail);
    AbortIfNot(data, fail);

    uint16_t command =  1 << 15 | ((uint16_t)reg) << 8;
    uint16_t response;
    AbortIfNot(transact_spi(adc->spi, command, &response), fail);

    *data = response & 0xFF;

    return success;
}
