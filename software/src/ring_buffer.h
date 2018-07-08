#ifndef PING_RING_BUFFER_H
#define PING_RING_BUFFER_H

#include "types.h"

/**
 * Describes a ping scheduled to occur in the future.
 */
typedef struct ping
{
    /**
     * The frequency of the ping.
     */
    PingFrequency frequency;

    /**
     * The expected timestamp when the ping should occur.
     */
    tick_t time;

} Ping;

/**
 * Holds a circular, queue-like structure for scheduled pings.
 */
typedef struct ring_buffer
{
    /**
     * The raw memory used by the ring buffer.
     */
    Ping data[10];

    /**
     * The start index of the ring buffer.
     */
    size_t head;

    /**
     * The end index of the ring buffer. The current end index is not a valid
     * element in the buffer.
     */
    size_t tail;

} ring_buffer_t;

result_t init_ring_buffer(ring_buffer_t *buffer);

result_t pop_ring_buffer(ring_buffer_t *buffer, Ping *element);

result_t push_ring_buffer(ring_buffer_t *buffer, const Ping element);

result_t length_ring_buffer(const ring_buffer_t *buffer, size_t *len);

#endif // PING_RING_BUFFER_H
