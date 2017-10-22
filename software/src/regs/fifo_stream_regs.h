#ifndef FIFO_STREAM_REGS_H
#define FIFO_STREAM_REGS_H

#include "types.h"

struct FifoStreamRegs
{
    uint32_t ISR;
    uint32_t UNUSED1[2];

    uint32_t TDFV;
    uint32_t UNUSED2[2];

    uint32_t RDFR;
    uint32_t RDFO;
    uint32_t RDFD;
    uint32_t RLR;
};

#endif
