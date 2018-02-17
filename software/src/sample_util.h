#ifndef SAMPLE_UTIL_H
#define SAMPLE_UTIL_H

#include "adc.h"
#include "dma.h"
#include "types.h"

result_t record(dma_engine_t *dma,
                sample_t *data,
                const size_t max_len,
                const adc_driver_t adc);

result_t acquire_sync(dma_engine_t *dma,
                      sample_t *data,
                      size_t max_len,
                      tick_t *ping_start,
                      bool *found,
                      analog_sample_t *max_value,
                      const adc_driver_t adc,
                      const uint32_t sampling_frequency,
                      HydroZynqParams *params,
                      filter_coefficients_t *filter,
                      const size_t filter_order);

result_t normalize(sample_t *data, const size_t len);

#endif
