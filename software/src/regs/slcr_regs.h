#ifndef SLCR_REGS_H
#define SLCR_REGS_H

#include "types.h"
#include "regs/defines.h"

struct SLCR_Regs
{
    uint32_t SCL;
    uint32_t SLCR_LOCK;
    uint32_t SLCR_UNLOCK;
    uint32_t SLCR_LOCKSTA;
    RESERVE(uint8_t, 0x200 - 0x10);
    uint32_t PSS_RST_CTRL;
};

static struct SLCR_Regs *SLCR = (struct SLCR_Regs *)(0xF8000000);

#endif
