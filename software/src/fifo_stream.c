#include "fifo_stream.h"

#include "abort.h"
#include "types.h"
#include "regs/fifo_stream_regs.h"

result_t init_fifo_stream(fifo_stream_t *fifo, uintptr_t base_addr)
{
    AbortIfNot(fifo, fail);
    AbortIfNot(base_addr, fail);

    /*
     * Set the register base address.
     */
    fifo->reg_base = (struct FifoStreamRegs *)base_addr;

    /*
     * Initialize, reset, and start the interface.
     */
    fifo->reg_base->RDFR = 0xA5;

    /*
     * Wait for the FIFO reset to complete.
     */
    while (!(fifo->reg_base->ISR & (1 << 23)));

    return success;
}

result_t has_packet(fifo_stream_t *fifo, bool *packet_available)
{
    AbortIfNot(fifo, fail);
    AbortIfNot(packet_available, fail);
    AbortIfNot(fifo->reg_base, fail);

    *packet_available = (fifo->reg_base->RDFO)? true : false;

    return success;
}

result_t get_word(fifo_stream_t *fifo, uint32_t *word)
{
    AbortIfNot(fifo, fail);
    AbortIfNot(word, fail);
    AbortIfNot(fifo->reg_base, fail);

    uint32_t count = fifo->reg_base->RLR;
    AbortIfNot(count % 4 == 0, fail);

    *word = fifo->reg_base->RDFD;

    return success;
}
