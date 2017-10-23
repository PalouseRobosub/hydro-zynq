#ifndef TRANSMISSION_UTIL_H
#define TRANSMISSION_UTIL_H

#include "types.h"

result_t send_result(upd_socket_t *socket, correlation_result_t *result);

result_t send_data(udp_socket_t *socket, sample_t *data, const size_t count);

#endif
