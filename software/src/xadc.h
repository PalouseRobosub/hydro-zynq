#ifndef XADC_H
#define XADC_H

#include "regs/XadcRegs.h"
#include "types.h"

typedef struct xadc_t
{
    struct XadcRegs *regs;
} xadc_t;

result_t init_xadc(xadc_t *xadc, uint32_t base_address);

result_t read_xadc_temperature(xadc_t *xadc, float *temp_c);

#endif
