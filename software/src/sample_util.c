#include "sample_util.h"
#include "correlation_util.h"
#include "abort.h"
#include "types.h"
#include "system.h"
#include "time_util.h"

result_t read_sample(fifo_stream_t *data_stream, sample_t *sample, tick_t timeout)
{
    AbortIfNot(data_stream, fail);
    AbortIfNot(sample, fail);

    /*
     * Read two words out of the fifo.
     */
    bool packet_available = false;
    const tick_t end_time = get_system_time() + timeout;
    while (!packet_available && get_system_time() < end_time)
    {
        AbortIfNot(has_packet(data_stream, &packet_available), fail);
    }

    AbortIfNot(packet_available, fail);

    uint32_t data[512];
    size_t len;
    if (get_packet(data_stream, data, 512, &len) == fail)
    {
        reset_fifo_stream(data_stream);
        AbortIfNot(read_sample(data_stream, sample, timeout), fail);
        return success;
    }

    if (len != 8)
    {
        uprintf("Got odd length (%d) packet.\n", len);
        return success;
    }

    /*
     * Translate the packet into a measurement.
     */
    sample->timestamp = get_system_time();
    sample->sample[0] = data[0] & 0xFFFF;
    sample->sample[1] = (data[0] >> 16) & 0xFFFF;
    sample->sample[2] = data[1] & 0xFFFF;
    sample->sample[3] = (data[1] >> 16) & 0xFFFF;

    return success;
}

result_t record(fifo_stream_t *data_stream,
                sample_t *data,
                const size_t max_len,
                size_t *num_samples,
                tick_t duration)
{
    AbortIfNot(data_stream, fail);
    AbortIfNot(data, fail);
    AbortIfNot(num_samples, fail);

    tick_t end_time = get_system_time() + duration;
    size_t i = 0;
    for (i = 0; i < max_len && get_system_time() < end_time; ++i)
    {
        AbortIfNot(read_sample(data_stream, &data[i], ms_to_ticks(1)), fail);
    }

    *num_samples = i;

    return success;
}

result_t acquire_sync(fifo_stream_t *data_stream,
                      sample_t *data,
                      const size_t max_len,
                      const tick_t duration,
                      tick_t *start_time)
{
    AbortIfNot(data_stream, fail);
    AbortIfNot(data, fail);
    AbortIfNot(start_time, fail);

    tick_t end_time = get_system_time() + duration;
    size_t samples = 0;
    while (get_system_time() < end_time && samples < max_len)
    {
        AbortIfNot(read_sample(data_stream, &data[samples++], ms_to_ticks(1)), fail);
    }

    filter_coefficients_t filter_coefficients;

    AbortIfNot(filter(data, samples, &filter_coefficients), fail);

    for (size_t i = 0; i < samples; ++i)
    {
        if (data[i].sample[0] > ADC_THRESHOLD)
        {
            *start_time = data[i].timestamp;
            return success;
        }
    }

    return fail;
}
