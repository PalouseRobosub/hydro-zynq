#ifndef CORRELATION_UTIL_H
#define CORRELATION_UTIL_H

#include "types.h"

result_t cross_correlate(const Sample *data, const size_t len, correlation_result_t *result);

result_t truncate(const Sample *data, const size_t len, size_t *start_index, size_t *end_index);

result_t filter(const Sample *data, const size_t len, filer_coefficient_t *coeffs);

#endif
