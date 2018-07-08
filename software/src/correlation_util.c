#include "correlation_util.h"

#include "types.h"
#include "abort.h"
#include "system_params.h"
#include "time_util.h"
#include "db.h"

result_t cross_correlate(const sample_t *data,
                         const size_t len,
                         correlation_t *correlations,
                         const size_t correlation_len,
                         size_t *num_correlations,
                         correlation_result_t *result,
                         const uint32_t sampling_frequency)
{
    AbortIfNot(data, fail);
    AbortIfNot(result, fail);
    AbortIfNot(len, fail);
    AbortIfNot(correlations, fail);
    AbortIfNot(num_correlations, fail);

    *num_correlations = 0;
    int32_t max_shift = MAX_SAMPLES_BETWEEN_PHONES;
    if (max_shift > len - 1)
    {
        max_shift = len - 1;
    }

    /*
     * Correlate the reference signal with channels A, B, and C.
     */
    for (int32_t lshift = max_shift; lshift > -1 * max_shift; lshift--)
    {
        /*
         * Grab the start and end indices of the unshifted signal for the
         * correlation.
         */
        (*num_correlations)++;
        size_t c_index = max_shift - lshift;
        correlations[c_index].left_shift = lshift;

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
        for (size_t k = 0; k< 4; ++ k)
        {
            correlations[c_index].result[k] = 0;
        }

        /*
         * Perform the actual correlation with the given channel sample
         * left-shift.
         */
        double correlation[3] = {0, 0, 0};
        for (size_t i = start_index; i < end_index; ++i)
        {
            for (size_t k = 0; k < 3; ++k)
            {
                correlation[k] += data[i].sample[0] * data[i + lshift].sample[k + 1];
            }
        }

        /*
         * Scale the analog raw data points to voltage readings to keep them in
         * range of a 32-bit number. Note that since we multiplied two raw
         * readings, we need to divide by two raw readings - hence the
         * multiplication.
         */
        for (size_t k = 0; k < 3; ++k)
        {
            correlations[c_index].result[k] = correlation[k] / ((2 << 13));
        }
    }

    /*
     * Loop through the results and find the maximum location of the correlation.
     */
    int32_t max_correlation_indices[3] = {0};
    for (int32_t i = max_shift; i > -1 * max_shift; --i)
    {
        int j = max_shift - i;
        for (size_t k = 0; k < 3; ++k)
        {
            if (correlations[j].result[k] > correlations[max_correlation_indices[k]].result[k])
            {
                max_correlation_indices[k] = j;
            }
        }
    }

    /*
     * Convert the max correlation index into a time measurement.
     */
    for (size_t i = 0; i < 3; ++i)
    {
        int32_t num_samples_right_shifted = -1 * correlations[max_correlation_indices[i]].left_shift;
        result->channel_delay_ns[i] = num_samples_right_shifted * 1000000000.0 / sampling_frequency;
    }

    return success;
}

size_t ticks_to_samples(tick_t ticks, const uint32_t sampling_frequency)
{
    return (size_t)(ticks * sampling_frequency / (float)CPU_CLOCK_HZ);
}

result_t truncate(const sample_t *data,
                  const size_t len,
                  size_t *start_index,
                  size_t *end_index,
                  bool *found,
                  const HydroZynqParams params,
                  const uint32_t sampling_frequency)
{
    AbortIfNot(data, fail);
    AbortIfNot(found, fail);
    AbortIfNot(start_index, fail);
    AbortIfNot(end_index, fail);

    size_t ping_start_index;
    *found = false;
    for (size_t i = 0; i < len; ++i)
    {
        for (size_t k = 0; k < 1; ++k)
        {
            if (!*found && data[i].sample[k] > params.ping_threshold)
            {
                ping_start_index = i;
                *found = true;
                break;
            }
        }

        if (*found)
        {
            break;
        }
    }

    if (!*found)
    {
        return success;
    }

    const size_t indices_before_start = ticks_to_samples(params.pre_ping_duration, sampling_frequency);
    if (indices_before_start > ping_start_index)
    {
        *start_index = 0;
    }
    else
    {
        *start_index = ping_start_index - indices_before_start;
    }

    *end_index = ping_start_index + ticks_to_samples(params.post_ping_duration, sampling_frequency);
    if (*end_index >= len)
    {
        *end_index = len - 1;
    }

    return success;
}

result_t filter(sample_t *data,
                const size_t len,
                filter_coefficients_t *coeffs,
                const size_t filter_order)
{
    AbortIfNot(data, fail);
    AbortIfNot(coeffs, fail);

    /*
     * TODO: Implement filtering algorithm.
     */
    for (size_t f = 0; f < filter_order; ++f)
    {
        const double reference_coefficient = coeffs[f].coefficients[3];

        /*
         * Normalize the filter coefficients by A0.
         */
        double coefficients[6];
        for (int i = 0; i < 6; ++i)
        {
            coefficients[i] = coeffs[f].coefficients[i] / reference_coefficient;
        }

        double z1[4] = {0}, z2[4] = {0};

        for (int i = 0; i < len; ++i)
        {
            for (int c = 0; c < 4; ++c)
            {
                const double old_value = data[i].sample[c];
                const double new_value = coefficients[0] * old_value + z1[c];

                /*
                 * Update the IIR filter state coefficients for each channel.
                 */
                z1[c] = coefficients[1] * old_value + z2[c] -
                            coefficients[4] * new_value;

                z2[c] = coefficients[2] * old_value -
                            coefficients[5] * new_value;


                /*
                 * Update the new measurement for the current sample after
                 * filtering.
                 */
                data[i].sample[c] = new_value;
            }
        }
    }


    return success;
}
