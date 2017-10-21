#ifndef GPIO_REGS_H
#define GPIO_REGS_H

#include "types.h"

struct GpioRegs
{
    uint32_t MASK_DATA_0_LSW;
    uint32_t MASK_DATA_0_MSW;
    uint32_t MASK_DATA_1_LSW;
    uint32_t MASK_DATA_1_MSW;
    uint32_t MASK_DATA_2_LSW;
    uint32_t MASK_DATA_2_MSW;
    uint32_t MASK_DATA_3_LSW;
    uint32_t MASK_DATA_3_MSW;
    uint32_t UNUSED3[8];
    uint32_t DATA[4];

    uint32_t UNUSED4[4];
    uint32_t DATA_RO[4];
    uint32_t UNUSED[101];

    uint32_t DIRM_0;
    uint32_t OEN_0;

    uint32_t INT_MASK_0;
    uint32_t INT_EN_0;
    uint32_t INT_DIS_0;
    uint32_t INT_STAT_0;
    uint32_t INT_TYPE_0;
    uint32_t INT_POLARITY_0;
    uint32_t INT_ANY_0;

    uint32_t UNUSED2[7];

    uint32_t DIRM_1;
    uint32_t OEN_1;
};

static struct GpioRegs *gpio_regs = (struct GpioRegs *)(0xe000a000);

#endif
