#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

typedef enum bool
{
    false,
    true
} bool;

/**
 * Defines the success and failure types.
 */
typedef enum result
{
    success = 1,
    fail = 0
} result_t;

/**
 * Defines an analog sample type.
 */
typedef int16_t analog_sample_t;

/**
 * Defines a timer tick type.
 */
typedef uint64_t tick_t;

#endif
