#ifndef DMA_H
#define DMA_H

#include "regs/DmaRegs.h"
#include "types.h"

typedef struct dma_engine_t
{
    struct DmaRegs *regs;
} dma_engine_t;

result_t initialize_dma(dma_engine_t *dma, uint32_t base_address);

result_t init_dma_transfer(dma_engine_t *dma, void *dest, uint32_t len);

result_t wait_for_dma_transfer(dma_engine_t *dma);

#endif
