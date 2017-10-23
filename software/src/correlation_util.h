#ifndef CORRELATION_UTIL_H
#define CORRELATION_UTIL_H

#include "types.h"

result_t cross_correlate(const sample_t *data, const size_t len, correlation_result_t *result);

result_t truncate(const sample_t *data, const size_t len, size_t *start_index, size_t *end_index, bool *found);

result_t filter(const sample_t *data, const size_t len, filter_coefficients_t *coeffs);

#endif
