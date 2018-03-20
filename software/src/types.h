#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stddef.h>

/**
 * Defines a boolean data type.
 */
typedef enum bool
{
    false = 0,
    true = 1
} bool;

/**
 * Defines a key-value pair combination.
 */
typedef struct KeyValuePair
{
    /*
     * A pointer to the key name of the pair.
     */
    char *key;

    /*
     * A pointer to the value portion of the pair.
     */
    char *value;

} KeyValuePair;

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
    /**
     * The analog sample measurement for each ADC channel.
     */
    analog_sample_t sample[4];

} sample_t;

/**
 * Structure that holes the result of a cross-correlation.
 */
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

/**
 * Specifies filter coefficients of an IIR filter.
 */
typedef struct filter_coefficients_t
{
    /**
     * Coefficieints are stored in form [B0, B1, B2, A0, A1, A2].
     */
    float coefficients[6];

} filter_coefficients_t;

/**
 * Specifies a pinger frequency.
 */
typedef enum PingFrequency
{
    /**
     * Unknown or unspecified pinger frequency.
     */
    Unknown,

    /**
     * Pinger frequency of approximately 25KHz.
     */
    TwentyFiveKHz,

    /**
     * Pinger frequency of approximately 30KHz.
     */
    ThirtyKHz,

    /**
     * Pinger frequency of approximately 35KHz.
     */
    ThirtyFiveKHz,

    /**
     * Pinger frequency of approximately 40KHz.
     */
    FourtyKHz

} PingFrequency;

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

    /**
     * Specified true if filtering should be completed on data.
     */
    bool filter;

    /**
     * Specifies the currently tracked pinger frequency.
     */
    PingFrequency primary_frequency;

} HydroZynqParams;

#endif
