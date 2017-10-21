#ifndef SYSTEM_REGS_H
#define SYSTEM_REGS_H

#include "types.h"

struct GlobalTimerRegs
{
    uint32_t Counter_Register[2];
    uint32_t Control_Register;
};

static struct GlobalTimerRegs *global_timer_regs = (struct GlobalTimerRegs *)(0xf8f00200);

#endif
