/**
 * HydroZynq Data acquisition firmware.
 *
 * @author Ryan Summers
 * @date 10/17/2017
 */

#include "abort.h"
#include "adc.h"
#include "correlation_util.h"
#include "dma.h"
#include "lwip/ip.h"
#include "lwip/udp.h"
#include "network_stack.h"
#include "sample_util.h"
#include "spi.h"
#include "system.h"
#include "system_params.h"
#include "time_util.h"
#include "transmission_util.h"
#include "types.h"
#include "udp.h"
#include "db.h"

#include "adc_dma_addresses.h"

/**
 * The DMA engine for reading samples.
 */
dma_engine_t dma;

/**
 * The SPI driver for controlling the ADC.
 */
spi_driver_t adc_spi;

/**
 * The ADC driver to use for controlling the ADC parameters.
 */
adc_driver_t adc;

/**
 * The maximum number of samples for 2.2 seconds at 65Msps.
 */
#define MAX_SAMPLES 45000 * 2200

/**
 * The array of current samples.
 */
sample_t samples[MAX_SAMPLES];

/**
 * The array of correlation results for the cross correlation.
 */
correlation_t correlations[50000];

/**
 * Specifies that the stream is in debug mode and transmits extra information.
 */
bool debug_stream = false;

/**
 * Specifies the current operating parameters of the application.
 */
HydroZynqParams params;

/**
 * Callback fro receiving a UDP packet.
 *
 * @return None.
 */
void receive_command(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, uint16_t port)
{
    debug_stream = !debug_stream;
    pbuf_free(p);
}

/**
 * Application process.
 *
 * @return Success or fail.
 */
result_t go()
{
    /*
     * Initialize the system.
     */
    AbortIfNot(init_system(), fail);

    dbprintf("Beginning HydroZynq main application\n");

    /*
     * Initialize the network stack with the specified IP address.
     */
    struct ip_addr our_ip, netmask, gateway;
    IP4_ADDR(&our_ip, 192, 168, 0, 10);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gateway, 192, 168, 1, 1);

    macaddr_t mac_address = {
        .addr = {0x00, 0x0a, 0x35, 0x00, 0x01, 0x02}
    };

    AbortIfNot(init_network_stack(our_ip, netmask, gateway, mac_address), fail);
    AbortIfNot(dbinit(), fail);
    dbprintf("Network stack initialized\n");

    /*
     * Initialize the DMA engine for reading samples.
     */
     AbortIfNot(initialize_dma(&dma, DMA_BASE_ADDRESS), fail);

    /*
     * Configure the ADC.
     */
    AbortIfNot(init_spi(&adc_spi, SPI_BASE_ADDRESS), fail);

    bool verify_write = false;
    bool use_test_pattern = false;
    AbortIfNot(init_adc(&adc, &adc_spi, ADC_BASE_ADDRESS, verify_write, use_test_pattern), fail);

    /*
     * Set the sample rate to 5MHz.
     */
    adc.regs->clk_div = 10;
    dbprintf("ADC clock div: %d\n", adc.regs->clk_div);
    dbprintf("ADC samples per packet: %d\n", adc.regs->samples_per_packet);

    /*
     * Bind the command port, data stream port, and the result output port.
     */
    udp_socket_t command_socket, data_stream_socket, result_socket, xcorr_stream_socket;

    struct ip_addr dest_ip;
    IP4_ADDR(&dest_ip, 192, 168, 0, 250);

    AbortIfNot(init_udp(&command_socket), fail);
    AbortIfNot(bind_udp(&command_socket, IP_ADDR_ANY, COMMAND_SOCKET_PORT, receive_command), fail);

    AbortIfNot(init_udp(&data_stream_socket), fail);
    AbortIfNot(connect_udp(&data_stream_socket, &dest_ip, DATA_STREAM_PORT), fail);

    AbortIfNot(init_udp(&xcorr_stream_socket), fail);
    AbortIfNot(connect_udp(&xcorr_stream_socket, &dest_ip, XCORR_STREAM_PORT), fail);

    AbortIfNot(init_udp(&result_socket), fail);
    AbortIfNot(connect_udp(&result_socket, &dest_ip, RESULT_PORT), fail);

    dbprintf("System initialization complete. Start time: %d ms\n",
            ticks_to_ms(get_system_time()));

    set_interrupts(true);

    /**
     * Set up the initial parameters.
     */
    params.sample_clk_div = adc.regs->clk_div;
    params.samples_per_packet = adc.regs->samples_per_packet;
    params.ping_threshold = INITIAL_ADC_THRESHOLD;

    bool sync = false;
    tick_t previous_ping_tick = get_system_time();
    while (1)
    {
        const uint32_t sampling_frequency = FPGA_CLK / (params.sample_clk_div * 2);

        /*
         * Push received network traffic into the network stack.
         */
        dispatch_network_stack();

        /*
         * Find sync for the start of a ping if we are not debugging.
         */
        if (!sync && !debug_stream)
        {
            bool found = false;
            int sync_attempts = 0;
            analog_sample_t max_value;
            while (!found)
            {
                uint32_t sample_duration_ms = 2100;
                uint32_t samples_to_take = sample_duration_ms / 1000.0 * sampling_frequency;
                AbortIfNot(acquire_sync(&dma,
                                        samples,
                                        samples_to_take,
                                        &previous_ping_tick,
                                        &found,
                                        &max_value,
                                        adc,
                                        sampling_frequency,
                                        params.ping_threshold), fail);

                /*
                 * Dispatch the network stack during sync to ensure messages
                 * are properly transmitted.
                 */
                dispatch_network_stack();

                if (!found)
                {
                    dbprintf("Failed to find ping during sync phase: %d - MaxVal: %d\n", ++sync_attempts, max_value);
                }
            }

            dbprintf("Synced: %f s - MaxVal: %d\n", ticks_to_seconds(previous_ping_tick), max_value);

            sync = true;
        }

        /*
         * Fast forward the previous ping tick until the most likely time
         * of the most recent ping.
         */
        if (!debug_stream)
        {
            tick_t next_ping_tick = previous_ping_tick;
            while (get_system_time() > (next_ping_tick - ms_to_ticks(50)))
            {
                next_ping_tick += ms_to_ticks(2000);
            }

            /*
             * Wait until the ping is about to come (100ms before).
             */
            while (get_system_time() < (next_ping_tick - ms_to_ticks(50)));
        }

        /*
         * Record the ping.
         */
        uint32_t sample_duration_ms = (debug_stream)? 300 : 300;
        uint32_t num_samples = sample_duration_ms / 1000.0 * sampling_frequency;
        if (num_samples % params.samples_per_packet)
        {
            num_samples += (params.samples_per_packet - (num_samples % params.samples_per_packet));
        }

        if (num_samples > MAX_SAMPLES)
        {
            num_samples = MAX_SAMPLES - (MAX_SAMPLES % adc.regs->samples_per_packet);
        }

        tick_t sample_start_tick = get_system_time();
        dbprintf("Sampling at: %f s\n", ticks_to_seconds(sample_start_tick));
        AbortIfNot(record(&dma, samples, num_samples, adc), fail);

        float seconds = ticks_to_seconds(get_system_time() - sample_start_tick);
        dbprintf("Sampling rate: %f Msps (%d in %f s)\n", num_samples / 1000000.0 / seconds, num_samples, seconds);

        AbortIfNot(normalize(samples, num_samples), fail);

        ///*
        // * Filter the received signal.
        // */
        //filter_coefficients_t coefficients;
        //AbortIfNot(filter(samples, num_samples, &coefficients), fail);

        /*
         * Truncate the data around the ping.
         */
        size_t start_index, end_index;
        bool located = false;
        AbortIfNot(truncate(samples, num_samples, &start_index, &end_index, &located, params.ping_threshold, sampling_frequency), fail);
        if (!located)
        {
            dbprintf("Failed to find the ping.\n");
            sync = false;
            if (debug_stream)
            {
                AbortIfNot(send_data(&data_stream_socket, samples, num_samples), fail);
            }
            continue;
        }
        else
        {
            tick_t offset = start_index * (CPU_CLOCK_HZ / sampling_frequency);
            previous_ping_tick = sample_start_tick + offset;
            dbprintf("Found ping: %f s\n", ticks_to_seconds(previous_ping_tick));

            /*
             * Locate the ping samples.
             */
            AbortIfNot(end_index > start_index, fail);
            sample_t *ping_start = &samples[start_index];
            size_t ping_length = end_index - start_index;

            /*
             * Perform the correlation on the data.
             */
            correlation_result_t result;
            size_t num_correlations;

            tick_t start_time = get_system_time();
            AbortIfNot(cross_correlate(ping_start, ping_length, correlations, MAX_SAMPLES * 2, &num_correlations, &result, sampling_frequency), fail);

            tick_t duration_time = get_system_time() - start_time;
            dbprintf("Correlation took %d ms\n", ticks_to_ms(duration_time));
            dbprintf("Correlation results: %d %d %d\n", result.channel_delay_ns[0], result.channel_delay_ns[1], result.channel_delay_ns[2]);

            /*
             * Relay the result.
             */
            //AbortIfNot(send_result(&result_socket, &result), fail);

            AbortIfNot(send_xcorr(&xcorr_stream_socket, correlations, num_correlations), fail);

            //TODO: DEBUG: Only show the correlation parts.
            AbortIfNot(send_data(&data_stream_socket, ping_start, ping_length), fail);
        }
    }
}

/**
 * Main entry point into the application.
 *
 * @return Zero upon success.
 */
int main()
{
    go();

    //give_up();

    while(1);
}
