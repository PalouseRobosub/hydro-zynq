#include "frequency_util.h"

#include "abort.h"
#include "db.h"
#include "math.h"
#include "system.h"
#include "time_util.h"
#include "types.h"

#include "fftw3.h"

/**
 * Allocate space for the input and output arrays to the FFT.
 */
fftw_complex input[10000], output[10000];

/**
 * Calculates the most prevalent frequency (in Hz) of a provided signal on the
 * first channel.
 *
 * @param[in] samples The input samples.
 * @param num_samples The number of samples provided in the input.
 * @param sample_frequency The sampling frequency of the input samples in Hz.
 * @param[out] frequency The most prevalent frequency detected in the signal.
 *
 * @return Success or fail.
 */
result_t get_frequency(const sample_t *samples,
                       const size_t num_samples,
                       const uint32_t sample_frequency,
                       float *frequency)
{
    AbortIfNot(samples, fail);
    AbortIfNot(num_samples, fail);
    AbortIfNot(frequency, fail);
    AbortIfNot(sample_frequency > 500000, fail);

    const size_t step_size = sample_frequency / 500000.0;
    AbortIfNot(step_size, fail);

    const size_t input_size = ((num_samples / step_size) > 10000)?
            10000 : num_samples / step_size;

    /*
     * The plan must be created before the input is initialized. The plan will
     * be executed later to actually perform the FFT.
     */
    fftw_plan plan = fftw_plan_dft_1d(input_size, input, output,
            FFTW_FORWARD, FFTW_ESTIMATE);

    for (int i = 0; i < input_size; ++i)
    {
        input[i][0] = samples[i * step_size].sample[0];
        input[i][1] = 0;
    }

    /*
     * Calculate the actual FFT.
     */
    const tick_t start_time = get_system_time();
    fftw_execute(plan);

    dbprintf("FFT took %lf seconds\n",
            ticks_to_seconds(get_system_time() - start_time));

    /*
     * From the FFT output, calculate the most prevalent frequency in the
     * signal.
     */
    size_t max_mag_index = 0;
    float max_mag = 0;
    for (int i = 0; i < input_size; ++i)
    {
        /*
         * Calculate the magnitude of the FFT bin.
         */
        const float mag = sqrt(output[i][0] * output[i][0] +
                               output[i][1] * output[i][1]);

        if (mag > max_mag)
        {
            max_mag = mag;
            max_mag_index = i;
        }
    }

    /*
     * Convert the max detected index into a real frequency.
     */
    *frequency = (sample_frequency / step_size) *
            (((float)max_mag_index) / input_size);

    dbprintf("Total FFT + analysis %lf seconds\n",
            ticks_to_seconds(get_system_time() - start_time));

    /*
     * Deallocate the plan.
     */
    fftw_destroy_plan(plan);

    return success;
}
