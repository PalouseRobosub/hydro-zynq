#ifndef SPI_DEVICE_H
#define SPI_DEVICE_H

#include "types.h"
#include "regs/SpiRegs.h"

typedef struct spi_driver_t
{
    struct SpiRegs * regs;
} spi_driver_t;

result_t init_spi(spi_driver_t *spi, uint32_t base_address);

result_t write_spi(spi_driver_t *spi, const uint16_t data);

result_t read_spi(spi_driver_t *spi, uint16_t *data);

result_t transact_spi(spi_driver_t *spi, const uint16_t data, uint16_t *response);

result_t disable_spi(spi_driver_t *spi);

result_t enable_spi(spi_driver_t *spi);

#endif
