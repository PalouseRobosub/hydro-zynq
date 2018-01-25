#include "db.h"

#include "types.h"
#include "abort.h"
#include "lwip/ip.h"
#include "udp.h"
#include "uart.h"
#include "system_params.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"

udp_socket_t db_socket;

result_t dbinit()
{
    AbortIfNot(init_udp(&db_socket), fail);

    struct ip_addr dest_ip;
    IP4_ADDR(&dest_ip, 192, 168, 0, 2);

    AbortIfNot(connect_udp(&db_socket, &dest_ip, DEBUG_PORT), fail);

    return success;
}

void dbprintf(char fmt[], ...)
{
    char str[256];
    va_list args;
    va_start(args, fmt);
    vsprintf(str, fmt, args);
    va_end(args);

    /*
     * Send debug data over the UDP port if it's configured.
     */
    if (db_socket.pcb)
    {
        send_udp(&db_socket, str, strlen(str));
    }

    size_t i = 0;
    while (str[i])
    {
        uart_putchar(str[i++]);
    }
}
