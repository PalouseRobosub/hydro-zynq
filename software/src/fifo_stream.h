#ifndef FIFO_STREAM_H
#define FIFO_STREAM_H

#include "types.h"

typedef struct fifo_stream_t
{
    struct FifoStreamRegs *reg_base;
} fifo_stream_t;

result_t init_fifo_stream(fifo_stream_t *fifo, uintptr_t base_addr);

result_t reset_fifo_stream(fifo_stream_t *fifo);

result_t get_fifo_packets(fifo_stream_t *fifo, uint32_t *packets);

result_t get_packet(fifo_stream_t *fifo, uint32_t *data, const size_t max_len, size_t *len);

#endif
