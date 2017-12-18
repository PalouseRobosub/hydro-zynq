#include "sample_util.h"
#include "correlation_util.h"
#include "abort.h"
#include "types.h"
#include "system.h"
#include "system_params.h"
#include "time_util.h"
#include "dma.h"
#include "xil_cache.h"

result_t record(dma_engine_t *dma,
                sample_t *data,
                const size_t sample_count)
{
    AbortIfNot(dma, fail);
    AbortIfNot(data, fail);
    AbortIfNot(sample_count % SAMPLES_PER_PACKET == 0, fail);

    size_t total_samples = 0;
    size_t invalid_packets = 0;
    while (total_samples < sample_count)
    {
        /*
         * Write back the cache lines before allowing DMA to avoid any
         * corruption when we later invalidate the cache lines.
         */
        Xil_DCacheFlushRange((INTPTR)&data[total_samples], 2 * 8 * SAMPLES_PER_PACKET);

        AbortIfNot(init_dma_transfer(dma, &data[total_samples], 2 * 8 * SAMPLES_PER_PACKET), fail);
        AbortIfNot(wait_for_dma_transfer(dma), fail);

        size_t samples = dma->regs->S2MM_LENGTH / 8;
        if (samples == SAMPLES_PER_PACKET)
        {
            /*
             * Invalidate cache lines associated with data samples. Beware that
             * this can potentially wipe out cached values stored _around_ the
             * buffer we are invalidating, so it should have been written back
             * earlier.
             */
            Xil_DCacheInvalidateRange((INTPTR)&data[total_samples], 2 * 8 * SAMPLES_PER_PACKET);
            total_samples += samples;
        }
        else
        {
            invalid_packets++;
        }
    }

    return success;
}

result_t normalize(sample_t *data, const size_t len)
{
    AbortIfNot(data, fail);

    /*
     * Accumulate the total value of each channel to find the average value.
     */
    uint64_t accumulators[4] = {0, 0, 0, 0};
    for (size_t i = 0; i < len; ++i)
    {
        for (size_t k = 0; k < 4; ++k)
        {
            accumulators[k] += data[i].sample[k];
        }
    }

    /*
     * Calculate the average of the channel as the offset.
     */
    analog_sample_t offset[4];
    for (size_t k = 0; k < 4; ++k)
    {
        offset[k] = accumulators[k] / len;
    }

    /*
     * Remove the average value from each sample.
     */
    for (size_t i = 0; i < len; ++i)
    {
        for (size_t k = 0; k < 4; ++k)
        {
            data[i].sample[k] -= offset[k];
        }
    }

    return success;
}

result_t acquire_sync(dma_engine_t *dma,
                      sample_t *data,
                      size_t max_len,
                      tick_t *start_time,
                      bool *found)
{
    AbortIfNot(dma, fail);
    AbortIfNot(data, fail);
    AbortIfNot(start_time, fail);
    AbortIfNot(found, fail);

    tick_t record_start = get_system_time();
    if (max_len % SAMPLES_PER_PACKET)
    {
        max_len -= max_len % SAMPLES_PER_PACKET;
    }

    AbortIfNot(record(dma, data, max_len), fail);
    AbortIfNot(normalize(data, max_len), fail);

    filter_coefficients_t filter_coefficients;

    AbortIfNot(filter(data, max_len, &filter_coefficients), fail);

    for (size_t i = 0; i < max_len; ++i)
    {
        for (size_t k = 0; k < 1; ++k)
        {
            if (data[i].sample[k] > ADC_THRESHOLD)
            {
                *start_time = record_start + i * (CPU_CLOCK_HZ / (float)SAMPLING_FREQUENCY);
                *found = true;
                return success;
            }
        }
    }

    *found = false;
    return success;
}
