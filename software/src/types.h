#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stddef.h>

typedef enum bool
{
    false = 0,
    true = 1
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

/**
 * Defines a single sample of an analog channel.
 */
typedef struct sample
{
    /*
     * The analog sample measurement for each ADC channel.
     */
    analog_sample_t sample[4];
} sample_t;

/**
 * Defines the correlation result type.
 */
typedef struct correlation_result_t
{
    /**
     * Specifies the time of flight delay in nanoseconds for each channel.
     */
    int32_t channel_delay_ns[3];

} correlation_result_t;

typedef struct filter_coefficients_t
{
    float coefficients[6];
} filter_coefficients_t;

#endif
