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

typedef struct correlation_t
{
    int32_t left_shift;
    int32_t result[3];
} correlation_t;

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

/**
 * Defines configurable parameters of the board.
 */
typedef struct HydroZynqParameters
{
    /**
     * Specifies the number of analog samples per DMA transfer.
     */
    uint32_t samples_per_packet;

    /**
     * Specifies the sampling frequency divider.
     */
    uint32_t sample_clk_div;

    /**
     * Specifies the threshold value that denotes a ping.
     */
    analog_sample_t ping_threshold;

    /*
     * Specifies the number of ticks before the threshold detection of a ping
     * to perform a correlation for.
     */
    tick_t pre_ping_duration;

    /**
     * Specifies the number of ticks after the threshold detection of a ping
     * to perform a correlation for.
     */
    tick_t post_ping_duration;

} HydroZynqParams;

#endif
