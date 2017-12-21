#include "uart.h"
#include "regs/uart_regs.h"

#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "system_params.h"
#include "abort.h"
#include "udp.h"
#include "lwip/ip.h"

udp_socket_t db_port;

result_t dbinit()
{
    AbortIfNot(init_udp(&db_port), fail);

    struct ip_addr dest_ip;
    IP4_ADDR(&dest_ip, 192, 168, 0, 250);

    AbortIfNot(connect_udp(&db_port, &dest_ip, DEBUG_PORT), fail);

    return success;
}

void uart_putchar(char c)
{
    /*
     * Wait for the UART FIFO to not be full.
     */
    while (uart_regs->Status_Register & (1 << 4));

    /*
     * Write the character to the UART FIFO.
     */
    uart_regs->TX_RX_FIFO = c;
}

void print_string(char *str)
{
    while (*str)
    {
        uart_putchar(*str++);
    }

    if (db_port.pcb)
    {
        send_udp(&db_port, str, strlen(str));
    }
}

void uprintf(const char fmt[], ...)
{
    char str[256];
    va_list args;
    va_start(args, fmt);
    vsprintf(str, fmt, args);
    va_end(args);

    print_string(str);
}
