#include "sample_util.h"

#include "abort.h"
#include "adc.h"
#include "correlation_util.h"
#include "dma.h"
#include "gpio.h"
#include "system.h"
#include "system_params.h"
#include "time_util.h"
#include "types.h"
#include "xil_cache.h"

/**
 * Records a number of analog samples.
 *
 * @param dma A pointer to the AXI DMA driver to use for sample acquisition.
 * @param gpio A pointer to the AXI GPIO driver to use for resetting the data
 *        FIFO.
 * @param data A pointer to where analog samples should be stored.
 * @param sample_count The number of samples to take.
 * @param adc The QuadADC driver that is connected to the DMA.
 *
 * @return Success or fail.
 */
result_t record(dma_engine_t *dma,
                gpio_driver_t *gpio,
                sample_t *data,
                const size_t sample_count,
                const adc_driver_t adc)
{
    AbortIfNot(dma, fail);
    AbortIfNot(data, fail);
    AbortIfNot(adc.regs, fail);
    AbortIfNot(sample_count % adc.regs->samples_per_packet == 0, fail);

    /*
     * Reset the FIFO feeding the DMA to purge stale data. The FIFO reset signal
     * needs to be asserted for atleast 3 of the slowest clock cycles. The
     * slowest clock is the FRAME_CLK at 5MHz. 1 microsecond should allow
     * atleast 5 clock cycles to pass.
     */
    AbortIfNot(gpio, fail);
    AbortIfNot(set_gpio(gpio, 0, false), fail);
    busywait(micros_to_ticks(1));
    AbortIfNot(set_gpio(gpio, 0, true), fail);

    size_t total_samples = 0;
    size_t invalid_packets = 0;
    while (total_samples < sample_count)
    {
        /*
         * Write back the cache lines before allowing DMA to avoid any
         * corruption when we later invalidate the cache lines.
         */
        Xil_DCacheFlushRange((INTPTR)&data[total_samples],
                             2 * 8 * adc.regs->samples_per_packet);

        AbortIfNot(init_dma_transfer(dma, &data[total_samples],
                    2 * 8 * adc.regs->samples_per_packet), fail);
        AbortIfNot(wait_for_dma_transfer(dma), fail);

        size_t samples = dma->regs->S2MM_LENGTH / 8;
        if (samples == adc.regs->samples_per_packet)
        {
            /*
             * Invalidate cache lines associated with data samples. Beware that
             * this can potentially wipe out cached values stored _around_ the
             * buffer we are invalidating, so it should have been written back
             * earlier using a cache flush opertion.
             */
            Xil_DCacheInvalidateRange((INTPTR)&data[total_samples],
                                      8 * samples);
            total_samples += samples;
        }
        else
        {
            invalid_packets++;
        }
    }

    return success;
}

/**
 * Normalize a number of samples.
 *
 * @note This function finds the mean for each channel and subtracts it from
 *       each measurement of the respective channel.
 *
 * @param data A pointer to the data to normalize.
 * @param len The length of samples to normalize.
 *
 * @return Success or fail.
 */
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

/**
 * Acquire sync with the ping.
 *
 * @param dma A pointer to the DMA driver to use for sampling.
 * @param gpio A pointer to the GPIO driver to use for resetting the data FIFO.
 * @param data A pointer to the location to store data.
 * @param max_len The maximum number of samples pointed to by data.
 * @param[out] start_time The tick that the ping started at.
 * @param[out] found Specified true if the ping was found.
 * @param[out] max_value The maximum value encountered on the reference channel.
 * @param adc The QuadADC driver used for acquiring samples.
 * @param sampling_frequency The sampling frequency of acquisition.
 * @param sample_threshold The threshold to use for ping detection.
 * @param filter The IIR filter to use for filtering received data.
 * @param filter_order The order of the IIR filter.
 *
 * @return Success or fail.
 */
result_t acquire_sync(dma_engine_t *dma,
                      gpio_driver_t *gpio,
                      sample_t *data,
                      size_t max_len,
                      tick_t *start_time,
                      bool *found,
                      analog_sample_t *max_value,
                      const adc_driver_t adc,
                      const uint32_t sampling_frequency,
                      HydroZynqParams *params,
                      filter_coefficients_t *iir_filter,
                      const size_t filter_order)
{
    AbortIfNot(dma, fail);
    AbortIfNot(gpio, fail);
    AbortIfNot(data, fail);
    AbortIfNot(params, fail);
    AbortIfNot(start_time, fail);
    AbortIfNot(found, fail);
    AbortIfNot(max_value, fail);
    AbortIfNot(adc.regs, fail);
    AbortIfNot(sampling_frequency, fail);

    const analog_sample_t sample_threshold = params->ping_threshold;

    tick_t record_start = get_system_time();
    if (max_len % adc.regs->samples_per_packet)
    {
        max_len -= max_len % adc.regs->samples_per_packet;
    }

    /*
     * Record and normalize the signal.
     */
    AbortIfNot(record(dma, gpio, data, max_len, adc), fail);
    AbortIfNot(normalize(data, max_len), fail);

    /*
     * Filter the received signal using the provided filter.
     */
    if (filter_order > 0 && params->filter)
    {
        AbortIfNot(filter(data, max_len, iir_filter, filter_order), fail);
    }

    /*
     * Search the filtered signal for the maximum value. If it is
     * above the threshold, indicate that sync was found.
     */
    *max_value = data[0].sample[0];

    for (size_t i = 0; i < max_len; ++i)
    {
        for (size_t k = 0; k < 1; ++k)
        {
            if (data[i].sample[k] > *max_value)
            {
                *max_value = data[i].sample[k];
            }

            if (data[i].sample[k] > sample_threshold)
            {
                *start_time = record_start +
                              i * (CPU_CLOCK_HZ / (float)sampling_frequency);
                *found = true;
                return success;
            }
        }
    }

    *found = false;
    return success;
}
