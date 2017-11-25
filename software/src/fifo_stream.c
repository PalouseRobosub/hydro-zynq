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

    fifo->reg_base->ISR = 0xFFFFFFFF;
    AbortIfNot(reset_fifo_stream(fifo), fail);

    return success;
}

result_t get_fifo_packets(fifo_stream_t *fifo, uint32_t *packets)
{
    AbortIfNot(fifo, fail);
    AbortIfNot(packets, fail);
    AbortIfNot(fifo->reg_base, fail);

    *packets = fifo->reg_base->RDFO;

    return success;
}

result_t reset_fifo_stream(fifo_stream_t *fifo)
{
    AbortIfNot(fifo, fail);

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

result_t get_packet(fifo_stream_t *fifo, uint32_t *data, const size_t max_len, size_t *len)
{
    AbortIfNot(fifo, fail);
    AbortIfNot(data, fail);
    AbortIfNot(len, fail);

    uint32_t packet_len = fifo->reg_base->RLR;
    if (max_len * 4 < packet_len)
    {
        return fail;
    }

    for (size_t i = 0; i < packet_len / 4; ++i)
    {
        data[i] = fifo->reg_base->RDFD;
    }

    *len = packet_len;
    return success;
}
