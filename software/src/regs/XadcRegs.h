#ifndef XADC_REGS_H
#define XADC_REGS_H

#include "regs/defines.h"
#include "types.h"

struct XadcRegs
{
    uint32_t SRR;
    uint32_t SR;
    uint32_t AOSR;
    uint32_t CONVSTR;
    uint32_t SYSMONRR;
    RESERVE(uint8_t, 0x200 - 4 * 5);
    uint32_t TEMP;
};

#endif
