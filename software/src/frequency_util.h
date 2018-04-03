#ifndef FREQUENCY_UTIL_H
#define FREQUENCY_UTIL_H

#include "types.h"
#include "udp.h"

result_t get_frequency(const sample_t *samples, const size_t num_samples, const uint32_t sampling_frequency, float *frequency, udp_socket_t *transmission_socket);

#endif // FREQUENCY_UTIL_H
