#include "ring_buffer.h"

#include "abort.h"
#include "db.h"
#include "types.h"

/**
 * Initializes a ring buffer.
 *
 * @param buffer The ring buffer to initialize.
 *
 * @return Success or fail.
 */
result_t init_ring_buffer(ring_buffer_t *buffer)
{
    AbortIfNot(buffer, fail);

    buffer->head = 0;
    buffer->tail = 0;

    return success;
}

/**
 * Increments a provided index for a ring buffer.
 *
 * @param buffer The ring buffer to use for reference.
 * @param index The index to increment.
 *
 * @return The properly wrapped newly incremented index.
 */
size_t ring_buffer_increment_index(const ring_buffer_t *buffer,
                                   const size_t index)
{
    AbortIfNot(buffer, fail);

    /*
     * Wrap the index back around to the start of the data if it exceeds the
     * max number of elements.
     */
    return (index + 1) % (sizeof(buffer->data) / sizeof(*buffer->data));
}

/**
 * Removes an element stored in the ring buffer.
 *
 * @param[in] buffer The ring buffer to remove from.
 * @param[out] element Optional. The location to store the popped element.
 *
 * @return Success if atleast one element can be popped, otherwise fail.
 */
result_t pop_ring_buffer(ring_buffer_t *buffer, Ping *element)
{
    AbortIfNot(buffer, fail);

    /*
     * Ensure that atleast one element exists in the ring buffer.
     */
    AbortIf(buffer->head == buffer->tail, fail);

    /*
     * If element is specified, store the popped element.
     */
    if (element)
    {
        *element = buffer->data[buffer->head];
    }

    /*
     * Advance the head index.
     */
    buffer->head = ring_buffer_increment_index(buffer, buffer->head);

    return success;
}

/**
 * Push a new element onto the ring buffer.
 *
 * @param buffer The location to store the new element.
 * @param element The new element to push to the ring buffer.
 *
 * @return Success if there is room to store the new element. Otherwise, fail.
 */
result_t push_ring_buffer(ring_buffer_t *buffer, const Ping element)
{
    AbortIfNot(buffer, fail);

    /*
     * Ensure there is room for atleast one more element.
     */
    AbortIf(ring_buffer_increment_index(buffer, buffer->tail) == buffer->head,
            fail);

    /*
     * Add the element to the tail and increment the index.
     */
    buffer->data[buffer->tail] = element;
    buffer->tail = ring_buffer_increment_index(buffer, buffer->tail);

    return success;
}

/**
 * Determines the length of data in a ring buffer.
 *
 * @param buffer The ring buffer to operate on.
 * @param[out] len Location to store the calculated length.
 *
 * @return Success or fail.
 */
result_t length_ring_buffer(const ring_buffer_t* buffer, size_t *len)
{
    AbortIfNot(buffer, fail);
    AbortIfNot(len, fail);

    if (buffer->head <= buffer->tail)
    {
        return buffer->tail - buffer->head;
    }
    else
    {
        return buffer->tail +
               (sizeof(buffer->data) / sizeof(*buffer->data) - buffer->head);
    }
}
