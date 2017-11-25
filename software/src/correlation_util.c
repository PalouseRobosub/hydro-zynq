#include "correlation_util.h"

#include "types.h"
#include "abort.h"
#include "system_params.h"
#include "time_util.h"

result_t cross_correlate(const sample_t *data,
                         const size_t len,
                         correlation_result_t *result)
{
    AbortIfNot(data, fail);
    AbortIfNot(result, fail);

    /*
     * Correlate the reference signal with channels A, B, and C.
     */
    int32_t correlation[MAX_SAMPLES * 2][3];
    for (int32_t lshift = (len - 1); lshift < (-1 * len); lshift--)
    {
        /*
         * Grab the start and end indices of the unshifted signal for the
         * correlation.
         */
        size_t start_index, end_index;
        if (lshift >= 0)
        {
            start_index = 0;
            end_index = len - lshift;
        }
        else
        {
            start_index = -1 * lshift;
            end_index = len;
        }

        /*
         * Set the initial correlation values to zero.
         */
        size_t correlation_index = (len - 1) - lshift;
        for (size_t k = 0; k< 4; ++ k)
        {
            correlation[correlation_index][k] = 0;
        }

        /*
         * Perform the actual correlation with the given channel sample
         * left-shift.
         */
        for (size_t i = start_index; i < end_index; ++i)
        {
            for (size_t k = 0; k < 3; ++k)
            {
                correlation[correlation_index][k] +=
                    data[i].sample[0] * data[i + lshift].sample[k + 1];
            }
        }
    }

    /*
     * Loop through the results and find the maximum location of the correlation.
     */
    int32_t max_correlation_indices[3] = {0};
    for (size_t i = 1; i < (len - 1) * 2; ++i)
    {
        for (size_t k = 0; k < 3; ++k)
        {
            if (correlation[i] > correlation[max_correlation_indices[k]])
            {
                max_correlation_indices[k] = i;
            }
        }
    }

    /*
     * Convert the max correlation index into a time measurement.
     */
    double sample_period_ns = 1000000000.0 / SAMPLING_FREQUENCY;
    for (size_t i = 0; i < 3; ++i)
    {
        int32_t num_samples_right_shifted = (len - 1) - max_correlation_indices[i];
        result->channel_delay_ns[i] = sample_period_ns * num_samples_right_shifted;
    }

    return success;
}

size_t ticks_to_samples(tick_t ticks)
{
    return (size_t)(ticks / ((float)CPU_CLOCK_HZ / SAMPLING_FREQUENCY));
}

result_t truncate(const sample_t *data,
                  const size_t len,
                  size_t *start_index,
                  size_t *end_index,
                  bool *found)
{
    AbortIfNot(data, fail);
    AbortIfNot(found, fail);
    AbortIfNot(start_index, fail);
    AbortIfNot(end_index, fail);

    size_t ping_start_index;
    *found = false;
    for (size_t i = 0; i < len; ++i)
    {
        for (size_t k = 0; k < 4; ++k)
        {
            if (!found && data[i].sample[k] > ADC_THRESHOLD)
            {
                ping_start_index = i;
                *found = true;
                break;
            }
        }

        if (found)
        {
            break;
        }
    }

    AbortIfNot(found, fail);

    size_t indices_before_start = ticks_to_samples(ms_to_ticks(1));
    size_t ping_start_truncated = ping_start_index;
    if (indices_before_start > ping_start_truncated)
    {
        ping_start_truncated = 0;
    }
    else
    {
        ping_start_truncated -= indices_before_start;
    }

    size_t indices_after_end = ping_start_index + ticks_to_samples(ticks_to_ms(5));
    if (indices_after_end >= len)
    {
        indices_after_end = len - 1;
    }

    *start_index = ping_start_truncated;
    *end_index = indices_after_end;

    return success;
}

result_t filter(const sample_t *data, const size_t len, filter_coefficients_t *coeffs)
{
    AbortIfNot(data, fail);
    AbortIfNot(coeffs, fail);

    /*
     * TODO: Implement filtering algorithm.
     */
    return success;
}
