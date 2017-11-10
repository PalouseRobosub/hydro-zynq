#include "uart.h"
#include "regs/uart_regs.h"

#include "stdarg.h"
#include "string.h"
#include "stdio.h"

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
