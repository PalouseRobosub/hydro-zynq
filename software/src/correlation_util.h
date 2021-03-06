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
        const HydroZynqParams params,
        const uint32_t sampling_frequency);

result_t filter(sample_t *data,
                const size_t len,
                filter_coefficients_t *coeffs,
                const size_t filter_order);

#endif
