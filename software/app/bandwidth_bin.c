#include "system.h"
#include "types.h"
#include "abort.h"
#include "time_util.h"
#include "sample_util.h"
#include "fifo_stream.h"
#include "spi.h"
#include "uart.h"

#include "adc_spi.h"
#include "adc.h"
#include "xsystem_monitor.h"
#include "regs/fifo_stream_regs.h"

result_t go()
{
    AbortIfNot(init_system(), fail);
    uprintf("System initialized.\n");

    /*
     * Initialize SPI with the device.
     */
    spi_driver_t spi;
    AbortIfNot(init_spi(&spi, SPI_BASE_ADDRESS), fail);

    /*
     * Initialize the ADC.
     */
    AbortIfNot(init_adc(&spi, 0), fail);

    /*
     * Set up and clear the FIFO stream.
     */
    fifo_stream_t stream;
    AbortIfNot(init_fifo_stream(&stream, FIFO_BASE_ADDRESS), fail);

    while (1)
    {
        /*
         * Read some samples.
         */
        const uint32_t sample_count = 1000000;
        const tick_t start_time = get_system_time();

        sample_t sample;
        uprintf("Reading\n");
        tick_t duration;
        for (size_t i = 0; i < sample_count; ++i)
        {
            volatile uint32_t available = 0;
            while (!available)
            {
                available = stream.reg_base->RDFO;
                if (!available)
                {
                    busywait(20);
                }
            }

            //uprintf(".");
            tick_t start = get_system_time();
            AbortIfNot(read_sample(&stream, &sample), fail);
            duration = get_system_time() - start;
        }

        const tick_t total_ticks = get_system_time() - start_time;

        float seconds = ticks_to_seconds(total_ticks);
        float mega_samples = sample_count / 1000000.0;
        uprintf("Bandwidth: %lf Msps/ch (%d samples in %lf seconds)\n", mega_samples / seconds, sample_count, seconds);
        uprintf("read_sample() took %d ticks\n", (int)duration);

    }
}

int main()
{
    go();

    while (1);
}
