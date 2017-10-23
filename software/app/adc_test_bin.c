#include "system.h"
#include "types.h"
#include "abort.h"
#include "time_util.h"
#include "sample_util.h"
#include "fifo_stream.h"
#include "uart.h"

int main()
{
    AbortIfNot(init_system(), fail);

    fifo_stream_t stream;

    AbortIfNot(init_fifo_stream(&stream, STREAM_FIFO_BASE_ADDRESS), fail);

    while (1)
    {
        /*
         * Wait until data is available in the FIFO.
         */
        sample_t sample;
        AbortIfNot(read_sample(&stream, &sample, ms_to_ticks(1)), fail);

        uprintf("Samples: 0: %x 1: %x 2: %x 3: %x\n", sample.sample[0], sample.sample[1], sample.sample[2], sample.sample[3]);
    }
}
