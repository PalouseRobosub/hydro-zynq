#include "dma.h"
#include "types.h"
#include "abort.h"
#include "regs/DmaRegs.h"
#include "network_stack.h"
#include "system.h"
#include "time_util.h"

result_t initialize_dma(dma_engine_t *dma, uint32_t base_address)
{
    AbortIfNot(dma, fail);
    dma->regs = (struct DmaRegs *)base_address;

    /*
     * Perform a DMA software reset and wait for it to complete.
     */
    dma->regs->S2MM_DMACR |= 1 << 2;
    while (dma->regs->S2MM_DMACR & (1 << 2));

    dma->regs->S2MM_DMACR |= 1;

    return success;
}

result_t init_dma_transfer(dma_engine_t *dma, void *dest, uint32_t len)
{
    AbortIfNot(dma, fail);
    AbortIfNot(dma->regs, fail);

    /*
     * Verify that the DMA engine has not encountered any internal errors.
     */
    AbortIf(dma->regs->S2MM_DMASR & (1 << 4), fail);

    /*
     * Verify DMA address alignment.
     */
    AbortIfNot((int)dest % 4 == 0, fail);

    /*
     * Verify that a positive number of bytes should be transfered.
     */
    AbortIfNot(len > 0 && len < (2 << (14 - 1)), fail);

    /*
     * Verify the DMA engine is started.
     */
    AbortIfNot((dma->regs->S2MM_DMACR & 1) == 1, fail);

    dma->regs->S2MM_DA = (uint32_t)dest;

    /*
     * The length register is written last to start the DMA transfer.
     */
    dma->regs->S2MM_LENGTH = len;

    return success;
}

result_t wait_for_dma_transfer(dma_engine_t *dma)
{
    AbortIfNot(dma, fail);
    AbortIfNot(dma->regs, fail);

    /*
     * Set a maximum end time.
     */
    const tick_t end_time = get_system_time() + ms_to_ticks(500);

    /*
     * Wait for the IDLE bit to be set.
     */
    while (get_system_time() < end_time &&
            dma->regs->S2MM_DMACR & 1 &&
            (dma->regs->S2MM_DMASR & (1 << 1)) == 0)
    {
        dispatch_network_stack();
    }

    AbortIf(get_system_time() >= end_time, fail);

    return success;
}
