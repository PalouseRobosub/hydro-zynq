#ifndef SAMPLE_UTIL_H
#define SAMPLE_UTIL_H

#include "types.h"
#include "dma.h"

result_t record(dma_engine_t *dma, sample_t *data, const size_t max_len);

result_t acquire_sync(dma_engine_t *dma, sample_t *data, const size_t max_len, tick_t *ping_start);

#endif
