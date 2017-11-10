#ifndef XSYSTEM_MONITOR_H
#define XSYSTEM_MONITOR_H

#include "regs/XSysMonRegs.h"
#include "types.h"

typedef struct xsystem_monitor_t
{
    struct XSysMonRegs *regs;
} xsystem_monitor_t;

result_t init_xsystem_monitor(xsystem_monitor_t *xadc, uint32_t base_address);

result_t read_fpga_temperature(xsystem_monitor_t *xadc, float *temp_c);

#endif
