#include "uart.h"
#include "regs/uart_regs.h"

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

void print(char *str)
{
    while (*str)
    {
        uart_putchar(*str++);
    }
}
