#include "frequency_util.h"

#include "abort.h"
#include "db.h"
#include "math.h"
#include "system.h"
#include "time_util.h"
#include "types.h"
#include "transmission_util.h"

#include "fft.h"

#define MAX_INPUT_LENGTH 8192
float input_data[MAX_INPUT_LENGTH * 2];

/**
 * Calculates the most prevalent frequency (in Hz) of a provided signal on the
 * first channel.
 *
 * @param[in] samples The input samples.
 * @param num_samples The number of samples provided in the input.
 * @param sample_frequency The sampling frequency of the input samples in Hz.
 * @param[out] frequency The most prevalent frequency detected in the signal.
 * @param[opt] transmission_socket UDP socket to transmit FFT result on.
 *
 * @return Success or fail.
 */
result_t get_frequency(const sample_t *samples,
                       const size_t num_samples,
                       const uint32_t sample_frequency,
                       float *frequency,
                       udp_socket_t *transmission_socket)
{
    AbortIfNot(samples, fail);
    AbortIfNot(num_samples, fail);
    AbortIfNot(frequency, fail);

    /*
     * Round the input size down to the nearest power of two.
     */
    size_t input_size = 1;
    while (input_size < num_samples)
    {
        input_size <<= 1;
    }

    input_size >>= 1;

    if (input_size > MAX_INPUT_LENGTH)
    {
        input_size = MAX_INPUT_LENGTH;
    }

    for (int i = 0; i < input_size; ++i)
    {
        input_data[i * 2] = samples[i].sample[0];
        input_data[i * 2 + 1] = 0;
    }

    /*
     * Calculate the actual FFT. Offset the pointer to input_data by 1 to
     * account for the algorithm using data starting at data[1] instead of
     * data[0].
     */
    AbortIfNot(discrete_fourier_transform(input_data - 1, input_size, 1), fail);

    /*
     * From the FFT output, calculate the most prevalent frequency in the
     * signal.
     */
    size_t max_mag_index = 0;
    float max_mag = 0;

    for (int i = 0; i < input_size / 2; ++i)
    {
        /*
         * Calculate the magnitude of the FFT bin.
         */
        const float mag = sqrt(input_data[i * 2] * input_data[i * 2] +
                               input_data[i * 2 + 1] * input_data[i * 2 + 1]);

        if (mag > max_mag)
        {
            max_mag = mag;
            max_mag_index = i;
        }
    }

    if (transmission_socket)
    {
        AbortIfNot(send_fft(transmission_socket,
                            input_data,
                            sample_frequency,
                            input_size),
                fail);
    }

    /*
     * Convert the max detected index into a real frequency.
     */
    *frequency = ((float)max_mag_index) / input_size * sample_frequency;

    return success;
}
