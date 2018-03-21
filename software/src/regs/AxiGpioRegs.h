#ifndef GPIO_REGS_H
#define GPIO_REGS_H

#include "types.h"
#include "regs/defines.h"

typedef struct AxiGpioRegs
{
    uint32_t GPIO_DATA;
    uint32_t GPIO_TRI;
    uint32_t GPIO2_DATA;
    uint32_t GPIO2_TRI;

    RESERVE(uint8_t, 0x11c - (0x0c + 4));
    uint32_t GIER;

    RESERVE(uint8_t, 0x120 - (0x11c + 4));
    uint32_t IP_IER;
    RESERVE(uint32_t, 1);
    uint32_t IP_ISR;
} AxiGpioRegs;

#endif // GPIO_REGS_H
