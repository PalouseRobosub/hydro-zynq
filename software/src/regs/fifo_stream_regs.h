#ifndef FIFO_STREAM_REGS_H
#define FIFO_STREAM_REGS_H

#include "types.h"

struct FifoStreamRegs
{
    volatile uint32_t ISR;
    volatile uint32_t UNUSED1[5];

    volatile uint32_t RDFR;
    volatile uint32_t RDFO;
    volatile uint32_t RDFD;
    volatile uint32_t RLR;
};

#endif
