#include "udp.h"

#include "abort.h"
#include "types.h"
#include "lwip/udp.h"
#include <string.h>

/**
 * Initializes a UDP socket.
 *
 * @return Success or fail.
 */
result_t init_udp(udp_socket_t *socket)
{
    AbortIfNot(socket, fail);

    socket->pcb = udp_new();

    AbortIfNot(socket->pcb, fail);

    return success;
}

/**
 * Deinitializes a UDP socket.
 *
 * @return Success or fail.
 */
result_t deinit_udp(udp_socket_t *socket)
{
    AbortIfNot(socket, fail);
    AbortIfNot(socket->pcb, fail);

    udp_remove(socket->pcb);
    socket->pcb = NULL;

    return success;
}

result_t sendto_udp(udp_socket_t *socket, struct ip_addr *ip, const uint16_t port, char *data)
{
    AbortIfNot(socket, fail);
    AbortIfNot(ip, fail);
    AbortIfNot(data, fail);

    struct pbuf *packet_buffer = pbuf_alloc(PBUF_TRANSPORT, strlen(data), PBUF_RAM);
    AbortIfNot(packet_buffer, fail);
    memcpy(packet_buffer->payload, data, strlen(data));

    result_t ret = udp_sendto(socket->pcb, packet_buffer, ip, port);

    pbuf_free(packet_buffer);

    return ((ret == ERR_OK)? success : fail);
}

result_t connect_udp(udp_socket_t *socket, struct ip_addr *ip, const uint16_t port)
{
    AbortIfNot(socket, fail);
    AbortIfNot(ip, fail);

    AbortIfNot(udp_connect(socket->pcb, ip, port) == ERR_OK, fail);

    return success;
}

result_t send_udp(udp_socket_t *socket, char *data)
{
    AbortIfNot(socket, fail);
    AbortIfNot(data, fail);

    struct pbuf *packet_buffer = pbuf_alloc(PBUF_TRANSPORT, strlen(data), PBUF_RAM);
    AbortIfNot(packet_buffer, fail);
    memcpy(packet_buffer->payload, data, strlen(data));

    result_t ret = udp_send(socket->pcb, packet_buffer);

    pbuf_free(packet_buffer);

    return ((ret == ERR_OK)? success : fail);
}

result_t bind_udp(udp_socket_t *socket, struct ip_addr *ip, uint16_t port, void (*recv)(void *arg, struct udp_pcb * upcb, struct pbuf *p, struct ip_addr *addr, uint16_t port))
{
    AbortIfNot(socket, fail);
    AbortIfNot(ip, fail);

    AbortIfNot(udp_bind(socket->pcb, ip, port) == ERR_OK, fail);

    if (recv)
    {
        udp_recv(socket->pcb, recv, NULL);
    }

    return success;
}
