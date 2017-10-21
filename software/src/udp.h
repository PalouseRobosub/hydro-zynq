#ifndef UDP_H
#define UDP_H

#include "lwip/udp.h"
#include "types.h"

typedef struct udp_socket_t
{
    struct udp_pcb *pcb;
} udp_socket_t;

result_t init_udp(udp_socket_t *socket);

result_t sendto_udp(udp_socket_t *socket, struct ip_addr *ip, const uint16_t port, char *data);

result_t deinit_udp(udp_socket_t *socket);

#endif
