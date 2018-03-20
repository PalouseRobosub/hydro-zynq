#ifndef FREQUENCY_UTIL_H
#define FREQUENCY_UTIL_H

#include "types.h"

result_t get_frequency(const sample_t *samples, const size_t num_samples, const uint32_t sampling_frequency, float *frequency);

#endif // FREQUENCY_UTIL_H
