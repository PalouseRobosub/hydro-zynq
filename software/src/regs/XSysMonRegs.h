#ifndef XADC_REGS_H
#define XADC_REGS_H

#include "regs/defines.h"
#include "types.h"

struct XSysMonRegs
{
    volatile uint32_t SRR;
    volatile uint32_t SR;
    volatile uint32_t AOSR;
    volatile uint32_t CONVSTR;
    volatile uint32_t SYSMONRR;
    RESERVE(uint8_t, 0x200 - 4 * 5);
    volatile uint32_t TEMP;
};

#endif
