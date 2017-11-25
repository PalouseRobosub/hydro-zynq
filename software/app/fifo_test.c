
#include "fifo_stream.h"
#include "regs/fifo_stream_regs.h"
#include "types.h"
#include "abort.h"
#include "system.h"
#include "sample_util.h"

#include "adc_final.h"

#include "uart.h"

result_t go()
{
    AbortIfNot(init_system(), fail);

    fifo_stream_t fifo;
    AbortIfNot(init_fifo_stream(&fifo, FIFO_BASE_ADDRESS), fail);

    tick_t lag_s = get_system_time();
    tick_t lag_e = get_system_time();

    const int lag = lag_e - lag_s;
    uprintf("System time latency: %d ticks\n", lag);

    volatile uint32_t available = 0;

    tick_t start = get_system_time();
    available = fifo.reg_base->RDFO;
    available = fifo.reg_base->RDFO;
    available = fifo.reg_base->RDFO;
    available = fifo.reg_base->RDFO;
    tick_t end = get_system_time();

    int ticks = end - start - lag;
    uprintf("Ticks (4x): %d\n", ticks);

    int i = 0;
    //sample_t sample;
    uint32_t data[512];
    while (1)
    {
        while (!available)
        {
            available = fifo.reg_base->RDFO;
        }

        start = get_system_time();
        uint32_t len = fifo.reg_base->RLR;
        for (size_t i = 0; i < len / 4; ++i)
        {
            data[i] = fifo.reg_base->RDFD;
        }

        //AbortIfNot(read_sample(&fifo, &sample), fail);
        end = get_system_time();

        ticks = end - start - lag;
        uprintf("Took %d ticks to read %d words.\n", ticks, len / 4);
        for (size_t i = 0; i < 0; ++i) //len / 4; ++i)
        {
            uprintf("- Data[%d]: %x\n", i, data[i]);
        }

        //uprintf("%d\n", i, sample.sample[0], sample.sample[1], sample.sample[2], sample.sample[3]);
        i++;
    }
}

int main()
{
    go();

    while (1);
}
