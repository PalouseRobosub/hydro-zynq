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

    /*
     * Set up the XADC for reading FPGA temperature.
     */
    xsystem_monitor_t system_monitor;
    AbortIfNot(init_xsystem_monitor(&system_monitor, XADC_BASE_ADDRESS), fail);

    /*
     * Enter blinky mode.
     */
    //while (1)
    //{
    //    float temp;
    //    AbortIfNot(read_fpga_temperature(&system_monitor, &temp), fail);
    //    uprintf("FPGA temperature = %f C\n", temp);

    //    /*
    //     * Write the on-board LED high.
    //     */
    //    set_board_led(true);
    //    busywait(ms_to_ticks(500));

    //    /*
    //     * Write the on-board LED low.
    //     */
    //    set_board_led(false);
    //    busywait(ms_to_ticks(500));
    //}

    while (1)
    {
        /*
         * Wait until data is available in the FIFO.
         */
        sample_t sample;
        AbortIfNot(read_sample(&stream, &sample, ms_to_ticks(10000)), fail);

        uprintf("Samples: 0: %x 1: %x 2: %x 3: %x\n", sample.sample[0], sample.sample[1], sample.sample[2], sample.sample[3]);
    }
}

int main()
{
    go();

    while (1);
}
