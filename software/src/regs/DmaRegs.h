#ifndef DMA_REGS_H
#define DMA_REGS_H

#include "regs/defines.h"
#include "types.h"

struct DmaRegs
{

    RESERVE(uint8_t, 0x30);
    volatile uint32_t S2MM_DMACR;
    volatile uint32_t S2MM_DMASR;
    RESERVE(uint8_t, 0x44 - 0x38 + 0x4);
    volatile uint32_t S2MM_DA;
    volatile uint32_t S2MM_DA_MSB;
    RESERVE(uint8_t, 0x58 - (0x4C + 0x4));
    volatile uint32_t S2MM_LENGTH;
};

#endif
