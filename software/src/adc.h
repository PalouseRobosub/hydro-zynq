#ifndef ADC_H
#define ADC_H

#include "spi.h"
#include "types.h"
#include "regs/AdcRegs.h"

typedef struct adc_driver_t
{
    spi_driver_t *spi;
    struct AdcRegs *regs;
} adc_driver_t;

result_t init_adc(adc_driver_t *adc, spi_driver_t *spi, uint32_t addr, bool verify, bool test_pattern);

result_t write_adc_register(adc_driver_t *adc, const uint8_t reg, uint8_t data);

result_t read_adc_register(adc_driver_t *adc, const uint8_t reg, uint8_t *data);

result_t write_verify_adc_register(adc_driver_t *adc,
                                   const uint8_t reg,
                                   uint8_t data,
                                   uint8_t expected_response);

#endif
