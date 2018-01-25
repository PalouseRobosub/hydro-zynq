#ifndef CORRELATION_UTIL_H
#define CORRELATION_UTIL_H

#include "types.h"

result_t cross_correlate(const sample_t *data,
                         const size_t len,
                         correlation_t *correlations,
                         const size_t correlation_len,
                         size_t *num_correlations,
                         correlation_result_t *result,
                         const uint32_t sampling_frequency);

result_t truncate(const sample_t *data,
        const size_t len,
        size_t *start_index,
        size_t *end_index,
        bool *found,
        const analog_sample_t threshold,
        const uint32_t sampling_frequency);

result_t filter(const sample_t *data,
                const size_t len,
                filter_coefficients_t *coeffs);

#endif
