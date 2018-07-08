#ifndef TRANSMISSION_UTIL_H
#define TRANSMISSION_UTIL_H

#include "types.h"
#include "udp.h"

result_t send_result(udp_socket_t *socket, correlation_result_t *result, const PingFrequency frequency);

result_t send_data(udp_socket_t *socket, sample_t *data, const size_t count);

result_t send_fft(udp_socket_t *socket, float *fft, const uint32_t sample_frequency, const size_t len);

result_t send_xcorr(udp_socket_t *socket, correlation_t *correlation, const size_t count);

result_t send_status_message(udp_socket_t *socket, const DeviceStatus *status, const HydroZynqParams *params);

#endif
