#include "abort.h"
#include "adc.h"
#include "dma.h"
#include "system.h"
#include "time_util.h"
#include "types.h"
#include "uart.h"
#include "xil_cache.h"

#include "adc_dma_gpio_addresses.h"

#define DMA_SAMPLE_SIZE 128
uint32_t samples[DMA_SAMPLE_SIZE * 2 * 2];

result_t go()
{
    AbortIfNot(init_system(), fail);
    Xil_DCacheDisable();

    dma_engine_t dma;
    AbortIfNot(initialize_dma(&dma, DMA_BASE_ADDRESS), fail);

    spi_driver_t adc_spi;
    AbortIfNot(init_spi(&adc_spi, SPI_BASE_ADDRESS), fail);

    adc_driver_t adc;
    AbortIfNot(init_adc(&adc, &adc_spi, ADC_BASE_ADDRESS, false, true), fail);

    adc.regs->clk_div = 10;
    adc.regs->samples_per_packet = 128;
    uprintf("CLK_DIV: %x\n", adc.regs->clk_div);
    uprintf("Samples /Packet: %x (%d)\n", adc.regs->samples_per_packet, adc.regs->samples_per_packet);

    uprintf("\nBeginning application.\n");

    uint32_t total_samples = 0;
    tick_t start_ticks = get_system_time();
    size_t odd_reads = 0;
    while (1)
    {
        AbortIfNot(init_dma_transfer(&dma, samples, DMA_SAMPLE_SIZE * 8), fail);
        AbortIfNot(wait_for_dma_transfer(&dma), fail);
        Xil_DCacheInvalidateRange((INTPTR)samples, DMA_SAMPLE_SIZE * 8);

        uint32_t samples = dma.regs->S2MM_LENGTH / 8;
        if (samples != adc.regs->samples_per_packet)
        {
            //uprintf("%d (expected %d)\n", samples, adc.regs->samples_per_packet);
            odd_reads++;
        }

        total_samples += samples;
        if (total_samples > 5000000)
        {
            tick_t end_ticks = get_system_time();
            tick_t total_ticks = end_ticks - start_ticks;
            float seconds = ticks_to_seconds(total_ticks);
            uprintf("Total rate: %f Msps (%d samples in %f secs) [Odd read count: %d]\n", total_samples / seconds / 1000000, total_samples, seconds, odd_reads);

            start_ticks = get_system_time();
            total_samples = 0;
            odd_reads = 0;
        }
    }
}

int main()
{
    go();

    while (1);
}
