#include "dma.h"

#include "abort.h"
#include "types.h"
#include "system.h"
#include "time_util.h"
#include "inttypes.h"

#include "regs/AdcRegs.h"

#include "adc_dma_addresses.h"

#include "db.h"

struct DmaRegs emulated_regs;

extern struct AdcRegs adc_regs;

bool initialized = false;

extern char _binary_samples_blob_start;
extern char _binary_samples_blob_end;
extern char _binary_samples_blob_size;

result_t initialize_dma(dma_engine_t *dma, uint32_t base_address)
{
    AbortIfNot(dma, fail);

    dma->regs = &emulated_regs;

    initialized = true;

    return success;
}

tick_t previous_dma = 0;
analog_sample_t *end_previous_transfer_index = (analog_sample_t *)&_binary_samples_blob_start;

result_t init_dma_transfer(dma_engine_t *dma, void *dest, uint32_t len)
{
    AbortIfNot(dma, fail);
    AbortIfNot(dest, fail);
    AbortIfNot(len, fail);

    if (len > adc_regs.samples_per_packet * sizeof(sample_t) / sizeof(analog_sample_t))
    {
        len = adc_regs.samples_per_packet * sizeof(sample_t) / sizeof(analog_sample_t);
    }

    /*
     * The emulator is not as fast as the real system. If the DMA
     * transfer is made up to 10ms after the previous one, assume
     * that the sampling is contiguous. Otherwise, sync up with
     * the system real time for getting samples.
     */
    analog_sample_t *data_index;

    if (ticks_to_seconds(get_system_time() - previous_dma) < 0.01)
    {
        data_index = end_previous_transfer_index;
    }
    else
    {
        float start_time = ticks_to_seconds(get_system_time());
        while (start_time >= 2.0)
        {
            start_time -= 2.0;
        }

        const size_t start_sample_index = start_time * 5000000.0;
        data_index = (analog_sample_t *)(&_binary_samples_blob_start + start_sample_index * sizeof(sample_t));
    }

    /*
     * Copy data into the buffer based on the call time.
     */
    for (int i = 0; i < len; ++i)
    {
        if (data_index >= (analog_sample_t *)(&_binary_samples_blob_start + (uint32_t)(&_binary_samples_blob_size)))
        {
            data_index = (analog_sample_t *)&_binary_samples_blob_start;
        }

        AbortIf(data_index < (analog_sample_t *)&_binary_samples_blob_start ||
                data_index > (analog_sample_t *)&_binary_samples_blob_end, fail);

        ((analog_sample_t *)dest)[i] = *data_index++;
    }

    /*
     * Update the DMA transfer length.
     */
    AbortIfNot(dma->regs, fail);
    dma->regs->S2MM_LENGTH = len * sizeof(analog_sample_t);

    previous_dma = get_system_time();
    end_previous_transfer_index = data_index;

    return success;
}

result_t wait_for_dma_transfer(dma_engine_t *dma)
{
    AbortIfNot(dma, fail);

    return success;
}
