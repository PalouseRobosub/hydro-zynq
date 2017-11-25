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
    while (total_samples < sample_count)
    {
        AbortIfNot(init_dma_transfer(dma, &data[total_samples], 2 * 8 * SAMPLES_PER_PACKET), fail);
        AbortIfNot(wait_for_dma_transfer(dma), fail);

        size_t samples = dma->regs->S2MM_LENGTH / 8;
        if (samples == SAMPLES_PER_PACKET)
        {
            total_samples += dma->regs->S2MM_LENGTH / 8;
        }
    }

    return success;
}

result_t acquire_sync(dma_engine_t *dma,
                      sample_t *data,
                      const size_t max_len,
                      tick_t *start_time)
{
    AbortIfNot(dma, fail);
    AbortIfNot(data, fail);
    AbortIfNot(start_time, fail);

    tick_t record_start = get_system_time();
    AbortIfNot(record(dma, data, max_len), fail);

    filter_coefficients_t filter_coefficients;

    AbortIfNot(filter(data, max_len, &filter_coefficients), fail);

    for (size_t i = 0; i < max_len; ++i)
    {
        if (data[i].sample[0] > ADC_THRESHOLD)
        {
            *start_time = record_start + i * CPU_CLOCK_HZ / (float)SAMPLING_FREQUENCY;
            return success;
        }
    }

    return fail;
}
