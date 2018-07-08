#include "spi.h"

#include "types.h"

result_t init_spi(spi_driver_t *spi, uint32_t base_address)
{
    spi->regs = (SpiRegs *)base_address;
    return success;
}

result_t write_spi(spi_driver_t *spi, const uint16_t data)
{
    return success;
}

result_t read_spi(spi_driver_t *spi, uint16_t *data)
{
    return success;
}

result_t transact_spi(spi_driver_t *spi, const uint16_t data, uint16_t *response)
{
    return success;
}

result_t disable_spi(spi_driver_t *spi)
{
    return success;
}

result_t enable_spi(spi_driver_t *spi)
{
    return success;
}
