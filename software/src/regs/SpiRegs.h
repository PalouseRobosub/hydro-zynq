#ifndef SPI_REGS_H
#define SPI_REGS_H

#include "types.h"
#include "regs/defines.h"

typedef struct SpiRegs
{
    RESERVE(uint8_t, 0x40);
    uint32_t SRR;
    RESERVE(uint8_t, 0x20 - 4);
    uint32_t SPICR;
    uint32_t SPISR;
    uint32_t SPI_DTR;
    uint32_t SPI_DRR;
    uint32_t SPISSR;
    uint32_t SPI_TX_OCC;
    uint32_t SPI_RX_OCC;
} SpiRegs;

#endif
