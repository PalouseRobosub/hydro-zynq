#ifndef UART_REGS_H
#define UART_REGS_H

#include "types.h"

struct UartRegs
{
    volatile uint32_t UNUSED[11];
    volatile uint32_t Status_Register;
    volatile uint32_t TX_RX_FIFO;
};

static struct UartRegs *uart_regs = (struct UartRegs *)(0xe0001000);

#endif
