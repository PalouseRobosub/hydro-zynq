#define max_samples 85000000

/*
 * Defines an analog sample type.
 */
typedef int16_t analog_sample_t;

/*
 * Defines a clock tick type.
 */
typedef uint64_t tick_t;

/*
 * Defines a single sample of an analog channel.
 */
typedef struct sample
{
    tick_t timestamp;
    analog_sample_t sample[4];

} Sample;

Sample samples[max_samples];

void acquire_sync()
{
    tick_t start_time = get_system_time();
    tick_t end_time = start_time + ms_to_ticks(2000);

    reset_fifo();

    while (get_system_time() < end_time && sample_cnt < max_samples)
    {
        samples[sample_cnt++] = read_sample();
    }

    filter(samples, sample_cnt, high_order_coefficients);

    for (size_t i = 0; i < sample_cnt; ++i)
    {
        if (samples[i].sample[0] > detection_threshold)
        {
            return samples[i].timestamp;
        }
    }

    return 0;
}

/**
 * Main entry point into the application.
 *
 * @return Zero upon success.
 */
int main()
{
    /*
     * Set up the network stack.
     */
    lwip_init();

    /*
     * Configure the ADC.
     */

    /*
     * Set up the FIFO.
     */

    /*
     * Bind the command port, data stream port, and the result output port.
     */

    bool sync = false;
    while (1)
    {
        /*
         * Find sync for the start of a ping.
         */
        if (!sync)
        {
            tick_t previous_ping_tick = 0;
            while (!previous_ping_tick)
            {
                previous_ping_tick = acquire_sync();
            }

            busywait(previous_ping_tick + ms_to_ticks(1900));
        }

        /*
         * Record the ping.
         */
        const uint32_t num_samples = record_for(samples, max_samples, ms_to_ticks(200));

        /*
         * Filter the signal.
         */
        filter(samples, num_samples);

        /*
         * Truncate the buffer around the ping.
         */
        size_t first_sample, last_sample;
        truncate(samples, &first_sample, &last_sample);

        size_t truncated_samples = last_sample - first_sample;

        get_start_time(&samples[first_sample], truncated_samples, &start_time);

        /*
         * Perform the correlation.
         */
        size_t xcorrs = xcorr(&samples[first_sample], truncated_samples,
                &xcorr_res[0], &xcorr_res[1], &xcorr_res[2], xcorr_length);

        /*
         * Find the maximum point of the cross correlation.
         */
        size_t max_indices[4] = {0, 0, 0, 0};
        double flight_delay[4];
        for (size_t i = 0; i < 4; ++i)
        {
            for (size_t j = 1; j < xcorrs; ++j)
            {
                if (xcorr_res[i][j] > xcorr_res[i][max_indices[i]])
                {
                    max_indices[i] = j;
                }
            }

            /*
             * Convert the maximum point into a time of flight difference.
             */
            const int sample_count_delta = xcorrs - 1 - max_indices[i];
            flight_delay[i] = sample_count_delta * sample_period;
        }

        /*
         * Relay the result.
         */
        transmit_result(flight_delay, 4);

        /*
         * If debug streams are enabled, transmit the data used for the
         * correlation.
         */
        if (debug_stream)
        {
            transmit_data(samples, num_samples);
            debug_stream = false;
        }
    }
}
