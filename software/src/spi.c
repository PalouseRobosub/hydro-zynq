#include "spi.h"

#include "types.h"
#include "abort.h"
#include "time_util.h"
#include "system.h"
#include "uart.h"

result_t init_spi(spi_driver_t *spi, uint32_t base_address)
{
    AbortIfNot(spi, fail);
    AbortIfNot(base_address, fail);

    spi->regs = (SpiRegs *)base_address;

    /*
     * Reset the SPI driver.
     */
    spi->regs->SRR = 0xa;

    /*
     * Configure the driver to transmit MSB first, clock idle
     * high, active edge is second clock edge, and reset the RX
     * and TX FIFOs.
     */
    uint16_t polarity = 0;
    uint16_t phase = 0;
    spi->regs->SPICR = 0x1E0 | phase << 4 | polarity << 3 | 0b110;

    /*
     * Set the SPI driver into loop-back mode and test for a proper read/write.
     */
    spi->regs->SPICR |= 1;

    uint16_t data = 0xab;
    uint16_t result = 0;
    AbortIfNot(transact_spi(spi, data, &result), fail);

    AbortIfNot(result == data, fail);

    /*
     * Set the SPI driver back into a normal mode.
     */
    spi->regs->SPICR &= ~1;

    return success;
}

result_t transact_spi(spi_driver_t *spi, const uint16_t data, uint16_t *response)
{
    AbortIfNot(spi, fail);
    AbortIfNot(spi->regs, fail);

    /*
     * Enable the slave-select to the SPI device.
     */
    spi->regs->SPISSR = 0;

    AbortIfNot(write_spi(spi, data), fail);
    AbortIfNot(read_spi(spi, response), fail);

    /*
     * De-assert the slave-select register.
     */
    spi->regs->SPISSR = 0xFFFFFFFF;

    return success;
}

result_t write_spi(spi_driver_t *spi, const uint16_t data)
{
    AbortIfNot(spi, fail);
    AbortIfNot(spi->regs, fail);

    /*
     * Inhibit SPI transmission while DTR is loaded.
     */
    spi->regs->SPICR |= 0x100;

    /*
     * Write data into the transmit register and wait for it to be sent.
     */
    spi->regs->SPI_DTR = data;

    /*
     * Enable data transmission.
     */
    spi->regs->SPICR &= ~(0x100);

    /*
     * Wait for the data to successfully transmit.
     */
    tick_t end = get_system_time() + ms_to_ticks(20);
    while (get_system_time() < end &&
           (!(spi->regs->SPISR & 0b100)));

    /*
     * Disable transmission now that we're done.
     */
    spi->regs->SPICR |= 0x100;

    /*
     * Check to see if a timeout occurred waiting for data to transmit.
     */
    AbortIfNot(get_system_time() < end, fail);

    return success;
}

result_t read_spi(spi_driver_t *spi, uint16_t *data)
{
    AbortIfNot(spi, fail);
    AbortIfNot(spi->regs, fail);
    AbortIfNot(data, fail);

    tick_t end = get_system_time() + ms_to_ticks(20);
    while ((spi->regs->SPISR & 0b1) && (get_system_time() < end));

    /*
     * Fail if there is no data in the receive FIFO.
     */
    AbortIf(spi->regs->SPISR & 0b1, fail);

    *data = spi->regs->SPI_DRR;

    return success;
}
