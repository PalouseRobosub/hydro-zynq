#ifndef SAMPLE_UTIL_H
#define SAMPLE_UTIL_H

#include "types.h"
#include "fifo_stream.h"

result_t read_sample(fifo_stream_t *data_stream, sample_t *sample, const tick_t duration);

result_t record(fifo_stream_t *data_stream, const sample_t *data, const size_t max_len, size_t *num_samples, tick_t duration);

result_t acquire_sync(fifo_stream_t *data_stream, sample_t *data, const size_t max_len, const tick_t duration, tick_t *start_time);

#endif
