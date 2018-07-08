/**
 * HydroZynq Data acquisition firmware.
 *
 * @author Ryan Summers
 * @date 10/17/2017
 */

#include "abort.h"
#include "adc.h"
#include "correlation_util.h"
#include "db.h"
#include "dma.h"
#include "gpio.h"
#include "frequency_util.h"
#include "lwip/ip.h"
#include "lwip/udp.h"
#include "network_stack.h"
#include "ring_buffer.h"
#include "sample_util.h"
#include "spi.h"
#include "system.h"
#include "system_params.h"
#include "time_util.h"
#include "transmission_util.h"
#include "types.h"
#include "udp.h"
#include "xsystem_monitor.h"

#include "adc_dma_reset_addresses.h"

#include <string.h>

/**
 * The FPGA XADC system monitor module for reading FPGA temperature.
 */
xsystem_monitor_t fpga_monitor;

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
 * The GPIO driver to use for controlling FIFO resets.
 */
gpio_driver_t gpio;

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
 * Specifies that the ping has been synced on.
 */
bool sync = false;

/**
 * Specifies that the ping frequency has been synced on.
 */
bool frequency_sync = false;

/**
 * Specifies the current operating parameters of the application.
 */
HydroZynqParams params;

/**
 * The current status of the device.
 */
DeviceStatus device_status;

/**
 * Defines the highpass IIR filter coefficients. These were generated by Matlab
 * using the designfilt function.
 */
filter_coefficients_t highpass_iir[5] = {
    {{0.976572753292004, -1.953145506584008, 0.976572753292004,
        1.000000000000000, -1.998354115074282, 0.998926104509836}},
    {{0.975206721477597, -1.950413442955194, 0.975206721477597,
        1.000000000000000, -1.995495119158081, 0.996193697294377}},
    {{0.972451482822301, -1.944902965644602, 0.972451482822301,
        1.000000000000000, -1.989660620860693, 0.990750529959661}},
    {{0.963669622248601, -1.927339244497202, 0.963669622248601,
        1.000000000000000, -1.970992420143032, 0.973473065140308}},
    {{0.906313647059524, -1.812627294119048, 0.906313647059524,
        1.000000000000000, -1.848974099452832, 0.860723515924862}}};

int frequency_to_int(PingFrequency f)
{
    switch (f)
    {
        case TwentyFiveKHz: return 25;
        case ThirtyKHz: return 30;
        case ThirtyFiveKHz: return 35;
        case FourtyKHz: return 40;
        default: return 0;
    }
}

/**
 * Parses an argument packet into key-value pairs.
 *
 * @param data The data to parse.
 * @param len The length of the memory pointed to by data.
 * @param[out] pairs A list of key-value pairs to store the parsed results in.
 * @param max_pairs The maximum number of key-value pairs allowed.
 * @param[out] num_pairs The number of key-value pairs found in the string.
 *
 * @return Success or fail.
 */
result_t parse_packet(char *data,
                      const size_t len,
                      KeyValuePair *pairs,
                      const size_t max_pairs,
                      size_t *num_pairs)
{
    AbortIfNot(data, fail);
    AbortIfNot(pairs, fail);
    AbortIfNot(num_pairs, fail);

    *num_pairs = 0;
    char *last_start = data;
    for (int i = 0; i < len && *num_pairs < max_pairs; ++i)
    {
        if (data[i] == ',' || data[i] == 0)
        {
            pairs[*num_pairs].key = last_start;
            data[i] = 0;
            (*num_pairs) = *num_pairs + 1;
            last_start = &(data[i + 1]);
        }

        if (data[i] == 0)
        {
            break;
        }
    }

    for (int i = 0; i < *num_pairs; ++i)
    {
        pairs[i].value = NULL;
        for (int j = 0; j < strlen(pairs[i].key); ++j)
        {
            if (pairs[i].key[j] == ':')
            {
                pairs[i].key[j] = 0;
                pairs[i].value = &(pairs[i].key[j + 1]);
                break;
            }
        }

        AbortIfNot(pairs[i].value, fail);
    }

    return success;
}

/**
 * Callback for receiving a UDP packet.
 *
 * @return None.
 */
void receive_command(void *arg,
                     struct udp_pcb *upcb,
                     struct pbuf *p,
                     struct ip_addr *addr,
                     uint16_t port)
{
    KeyValuePair pairs[10];
    size_t num_entries = 0;

    char data[1024];
    if (p->len > (sizeof(data) - 1))
    {
        pbuf_free(p);
        dbprintf("Packet too long! Length was %d but internal buffer is %d\n",
                p->len, sizeof(data));
        return;
    }

    memcpy(data, p->payload, p->len);
    pbuf_free(p);
    data[p->len] = 0;

    AbortIfNot(parse_packet(data, p->len + 1, pairs, 10, &num_entries), );

    for (int i = 0; i < num_entries; ++i)
    {
        dbprintf("Key: '%s' Value: '%s'\n", pairs[i].key, pairs[i].value);
        if (strcmp(pairs[i].key, "threshold") == 0)
        {
            unsigned int threshold;
            AbortIfNot(sscanf(pairs[i].value, "%u", &threshold), );
            params.ping_threshold = threshold;
            sync = false;
            dbprintf("Ping threshold has been set to %d\n", params.ping_threshold);
        }
        else if (strcmp(pairs[i].key, "filter") == 0)
        {
            unsigned int debug = 0;
            AbortIfNot(sscanf(pairs[i].value, "%u", &debug), );
            params.filter = (debug == 0)? false : true;
            dbprintf("Filtering is: %s\n",
                    (debug_stream)? "Enabled" : "Disabled");
        }
        else if (strcmp(pairs[i].key, "debug") == 0)
        {
            unsigned int debug = 0;
            AbortIfNot(sscanf(pairs[i].value, "%u", &debug), );
            debug_stream = (debug == 0)? false : true;
            dbprintf("Debug stream is: %s\n",
                    (debug_stream)? "Enabled" : "Disabled");
        }
        else if (strcmp(pairs[i].key, "pre_ping_duration_us") == 0)
        {
            unsigned int duration = 0;
            AbortIfNot(sscanf(pairs[i].value, "%u", &duration), );

            params.pre_ping_duration = micros_to_ticks(duration);
            dbprintf("Pre-ping duration is %u us.\n", duration);
        }
        else if (strcmp(pairs[i].key, "post_ping_duration_us") == 0)
        {
            unsigned int duration = 0;
            AbortIfNot(sscanf(pairs[i].value, "%u", &duration), );

            params.post_ping_duration = micros_to_ticks(duration);
            dbprintf("Post-ping duration is %u us.\n", duration);
        }
        else if (strcmp(pairs[i].key, "frequency") == 0)
        {
            unsigned int frequency = 0;
            AbortIfNot(sscanf(pairs[i].value, "%u", &frequency), );

            if (frequency == 25000)
            {
                params.primary_frequency = TwentyFiveKHz;
            }
            else if (frequency == 30000)
            {
                params.primary_frequency = ThirtyKHz;
            }
            else if (frequency == 35000)
            {
                params.primary_frequency = ThirtyFiveKHz;
            }
            else if (frequency == 40000)
            {
                params.primary_frequency = FourtyKHz;
            }
            else
            {
                dbprintf("Unkown frequency specified: %u\n", frequency);
                dbprintf("Please use one of the following: 25000, ");
                dbprintf("30000, 35000, 40000\n");
                AbortIfNot(fail, );
            }

            frequency_sync = false;

            dbprintf("Primary frequency target is now: %d\n", frequency);
        }
        else if (strcmp(pairs[i].key, "track") == 0)
        {
            unsigned int frequency = 0;
            AbortIfNot(sscanf(pairs[i].value, "%u", &frequency), );

            if (frequency == 25000)
            {
                params.track_25khz = true;
            }
            else if (frequency == 30000)
            {
                params.track_30khz = true;
            }
            else if (frequency == 35000)
            {
                params.track_35khz = true;
            }
            else if (frequency == 40000)
            {
                params.track_40khz = true;
            }
            else
            {
                dbprintf("Unkown frequency specified: %u\n", frequency);
                dbprintf("Please use one of the following: 25000, ");
                dbprintf("30000, 35000, 40000\n");
                AbortIfNot(fail, );
            }

            frequency_sync = false;
            dbprintf("Frequency is now tracked: %d\n", frequency);
        }
        else if (strcmp(pairs[i].key, "untrack") == 0)
        {
            unsigned int frequency = 0;
            AbortIfNot(sscanf(pairs[i].value, "%u", &frequency), );

            if (frequency == 25000)
            {
                params.track_25khz = false;
            }
            else if (frequency == 30000)
            {
                params.track_30khz = false;
            }
            else if (frequency == 35000)
            {
                params.track_35khz = false;
            }
            else if (frequency == 40000)
            {
                params.track_40khz = false;
            }
            else
            {
                dbprintf("Unkown frequency specified: %u\n", frequency);
                dbprintf("Please use one of the following: 25000, ");
                dbprintf("30000, 35000, 40000\n");
                AbortIfNot(fail, );
            }

            dbprintf("Frequency is no longer tracked: %d\n", frequency);
        }
        else if (strcmp(pairs[i].key, "reset") == 0)
        {
            /*
             * Trigger a software reset of the Zynq.
             */
            dbprintf("Resetting Zynq...");
            give_up();
        }
    }
}

/**
 * Issue a request for thruster silent running for clean hydrophone readings.
 *
 * @param socket The socket to make the request on.
 * @param ticks The time in the future that thrusters should be shut down at.
 * @param duration The duration that thrusters should be shut down for.
 *
 * @return Success or fail.
 */
result_t request_thruster_shutdown(udp_socket_t *socket,
                                   const tick_t future_ticks,
                                   const tick_t duration)
{
    char msg[8];
    int32_t when_ms = ticks_to_ms(future_ticks - get_system_time());
    int32_t duration_ms = ticks_to_ms(duration);
    memcpy(msg, &when_ms, 4);
    memcpy(&msg[4], &duration_ms, 4);

    #ifndef EMULATED
        AbortIfNot(send_udp(socket, msg, 8), fail);
    #endif

    return success;
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
    IP4_ADDR(&our_ip, 192, 168, 0, 7);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gateway, 192, 168, 1, 1);

    macaddr_t mac_address = {
        .addr = {0x00, 0x0a, 0x35, 0x00, 0x01, 0x02}
    };

    AbortIfNot(init_network_stack(our_ip, netmask, gateway, mac_address), fail);

    #ifndef EMULATED
        AbortIfNot(dbinit(), fail);
    #endif

    dbprintf("Network stack initialized\n");

    /*
     * Initialize the FPGA XADC for temperature monitoring.
     */
    AbortIfNot(init_xsystem_monitor(&fpga_monitor, XADC_BASE_ADDRESS), fail);

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
     * Initialize the GPIO reset driver. Bring the FIFO out of reset.
     */
    AbortIfNot(init_gpio(&gpio, GPIO_BASE_ADDRESS, 1), fail);
    AbortIfNot(set_gpio(&gpio, 0, true), fail);

    /*
     * Bind the command port, data stream port, and the result output port.
     */
    udp_socket_t command_socket, data_stream_socket, result_socket, xcorr_stream_socket, silent_request_socket;

    udp_socket_t fft_socket, status_socket;

    struct ip_addr dest_ip;
    IP4_ADDR(&dest_ip, 192, 168, 0, 2);

    AbortIfNot(init_udp(&command_socket), fail);
    AbortIfNot(bind_udp(&command_socket, IP_ADDR_ANY, COMMAND_SOCKET_PORT, receive_command), fail);

    AbortIfNot(init_udp(&silent_request_socket), fail);
    AbortIfNot(connect_udp(&silent_request_socket, &dest_ip, SILENT_REQUEST_PORT), fail);

    AbortIfNot(init_udp(&data_stream_socket), fail);
    AbortIfNot(connect_udp(&data_stream_socket, &dest_ip, DATA_STREAM_PORT), fail);

    AbortIfNot(init_udp(&xcorr_stream_socket), fail);
    AbortIfNot(connect_udp(&xcorr_stream_socket, &dest_ip, XCORR_STREAM_PORT), fail);

    AbortIfNot(init_udp(&result_socket), fail);
    AbortIfNot(connect_udp(&result_socket, &dest_ip, RESULT_PORT), fail);

    AbortIfNot(init_udp(&fft_socket), fail);
    AbortIfNot(connect_udp(&fft_socket, &dest_ip, FFT_RESULT_PORT), fail);

    AbortIfNot(init_udp(&status_socket), fail);
    AbortIfNot(connect_udp(&status_socket, &dest_ip, DEVICE_STATUS_PORT), fail);

    dbprintf("System initialization complete. Start time: %d ms\n",
            ticks_to_ms(get_system_time()));

    set_interrupts(true);

    /*
     * Read the first sample from the ADC and ignore it - the ADC always has an
     * invalid measurement as the first reading.
     */
    AbortIfNot(record(&dma, &gpio, samples, adc.regs->samples_per_packet, adc), fail);

    /*
     * Set up the initial parameters.
     */
    params.sample_clk_div = adc.regs->clk_div;
    params.samples_per_packet = adc.regs->samples_per_packet;
    params.ping_threshold = INITIAL_ADC_THRESHOLD;

    /*
     * Track all frequencies.
     */
    params.primary_frequency = TwentyFiveKHz;
    params.track_25khz = true;
    params.track_30khz = true;
    params.track_35khz = true;
    params.track_40khz = true;

    /*
     * Perform a correlation for two wavelengths after the threshold is
     * encountered. Before the threshold will be either the initial wavefronts
     * or white noise floor. Because the noise floor is small, it will not
     * affect the correlation.
     */
    params.pre_ping_duration = micros_to_ticks(100);
    params.post_ping_duration = micros_to_ticks(50);
    params.filter = false;

    /*
     * Set up initial device status parameters. Firmware revision
     * is hard-defined by the make script.
     */
    device_status.firmware_rev = FIRMWARE_REVISION;
    device_status.correlation_time_us = 0;
    device_status.fpga_temp_f = 0;
    device_status.fft_time_us = 0;
    device_status.fft_frequency_hz = 0;
    device_status.normalize_record_time_us = 0;
    device_status.ping_processing_time_us = 0;

    ring_buffer_t future_ping_queue;
    AbortIfNot(init_ring_buffer(&future_ping_queue), fail);
    while (1)
    {
        const uint32_t sampling_frequency = FPGA_CLK /
                                            (params.sample_clk_div * 2);
        const PingFrequency tracked_frequency = params.primary_frequency;

        float fpga_temperature_c = 0;
        AbortIfNot(read_fpga_temperature(&fpga_monitor, &fpga_temperature_c), fail);

        device_status.sampling_frequency = sampling_frequency;
        device_status.fpga_temp_f = fpga_temperature_c * 1.8 + 32;

        /*
         * Send out a telemetry message indicating device status.
         */
        AbortIfNot(send_status_message(&status_socket, &device_status, &params), fail);

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
            tick_t previous_ping_tick;
            while (!found && !debug_stream)
            {
                uint32_t sample_duration_ms = 2100;
                uint32_t samples_to_take = sample_duration_ms / 1000.0 *
                                           sampling_frequency;

                AbortIfNot(acquire_sync(&dma,
                                        &gpio,
                                        samples,
                                        samples_to_take,
                                        &previous_ping_tick,
                                        &found,
                                        &max_value,
                                        adc,
                                        sampling_frequency,
                                        &params,
                                        highpass_iir,
                                        5), fail);

                /*
                 * Dispatch the network stack during sync to ensure messages
                 * are properly transmitted.
                 */
                dispatch_network_stack();

                if (!found)
                {
                    dbprintf("Failed to find ping during sync "
                             "phase: %d - MaxVal: %d\n",
                                ++sync_attempts,
                                max_value);
                }
            }

            if (found)
            {
                dbprintf("Synced: %f s - MaxVal: %d\n",
                        ticks_to_seconds(previous_ping_tick),
                        max_value);
                sync = true;
                frequency_sync = false;

                const Ping new_ping = {
                    .frequency = Unknown,
                    .time = previous_ping_tick + ms_to_ticks(2000)
                };

                AbortIfNot(init_ring_buffer(&future_ping_queue), fail);
                AbortIfNot(push_ring_buffer(&future_ping_queue, new_ping),
                    fail);
            }
        }

        /*
         * Fast forward the previous ping tick until the most likely time
         * of the most recent ping.
         */
        Ping current_ping;
        if (!debug_stream)
        {
            /*
             * Figure out when the next ping should be arriving.
             */
            AbortIfNot(pop_ring_buffer(&future_ping_queue, &current_ping),
                fail);

            /*
             * If it is too late to record the ping we just removed from the
             * queue, cycle forward in the queue or fast forward our timestamp
             * if there are no other queued pings.
             */
            while ((current_ping.time - ms_to_ticks(25)) < get_system_time())
            {
                /*
                 * Never cycle past the frequency of interest. We have to
                 * guarantee that we measure this ping atleast once every two
                 * ping cycles. Additionally, do not cycle forward if there are
                 * no more scheduled pings available in the queue.
                 */
                size_t queue_length = 0;
                AbortIfNot(length_ring_buffer(&future_ping_queue,
                                              &queue_length),
                    fail);

                if (current_ping.frequency == params.primary_frequency ||
                    queue_length == 0)
                {
                    dbprintf("%4.5lf: Missed %d KHz primary ping @ %4.5lf - waiting for next\n",
                            ticks_to_seconds(get_system_time()),
                            frequency_to_int(current_ping.frequency),
                            ticks_to_seconds(current_ping.time));
                    current_ping.time += ms_to_ticks(2000);
                }
                else
                {
                    /*
                     * We missed this ping. Set a timer for the next one in the
                     * future and enqueue it.
                     */
                    while (current_ping.time < get_system_time())
                    {
                        current_ping.time += ms_to_ticks(2000);
                    }

                    /*
                     * Add a final 2 seconds to ensure it is
                     * scheduled past the primary ping.
                     */
                    current_ping.time += ms_to_ticks(2000);

                    AbortIfNot(push_ring_buffer(&future_ping_queue,
                                                current_ping),
                        fail);
                    dbprintf("%4.5lf: Missed %d KHz secondary ping. Next scheduled for %4.5lf\n",
                            ticks_to_seconds(get_system_time()),
                            frequency_to_int(current_ping.frequency),
                            ticks_to_seconds(current_ping.time));

                    /*
                     * Cycle to the next scheduled ping.
                     */
                    AbortIfNot(pop_ring_buffer(&future_ping_queue,
                                               &current_ping),
                        fail);
                }
            }

            /*
             * Request that thrusters enter shutdown at the next ping tick.
             */
            AbortIfNot(request_thruster_shutdown(
                        &silent_request_socket,
                        (current_ping.time - ms_to_ticks(25)),
                        ms_to_ticks(50)), fail);

            /*
             * Wait until the ping is about to come before recording.
             */
            while (get_system_time() < (current_ping.time - ms_to_ticks(25)));
        }

        /*
         * Record the ping.
         */
        uint32_t sample_duration_ms = (debug_stream)? 2100 : 100;
        uint32_t num_samples = sample_duration_ms / 1000.0 * sampling_frequency;
        if (num_samples % params.samples_per_packet)
        {
            num_samples += (params.samples_per_packet -
                    (num_samples % params.samples_per_packet));
        }

        if (num_samples > MAX_SAMPLES)
        {
            num_samples = MAX_SAMPLES -
                (MAX_SAMPLES % adc.regs->samples_per_packet);
        }

        tick_t sample_start_tick = get_system_time();
        AbortIfNot(record(&dma, &gpio, samples, num_samples, adc), fail);

        AbortIfNot(normalize(samples, num_samples), fail);

        device_status.normalize_record_time_us = ticks_to_us(get_system_time() -
                                                 sample_start_tick);

        /*
         * Filter the received signal.
         */
        if (params.filter)
        {
            const tick_t filter_start_time = get_system_time();
            AbortIfNot(filter(samples, num_samples, highpass_iir, 5), fail);

            dbprintf("Filtering took %lf seconds.\n",
                    ticks_to_seconds(get_system_time() - filter_start_time));
        }

        /*
         * If debugging is enabled, don't perform the correlation or truncation
         * steps and just dump data.
         */
        if (debug_stream)
        {
            AbortIfNot(send_data(&data_stream_socket, samples, num_samples),
                fail);
            continue;
        }

        /*
         * Truncate the data around the ping.
         */
        size_t start_index, end_index;
        bool located = false;
        AbortIfNot(truncate(samples,
                            num_samples,
                            &start_index,
                            &end_index,
                            &located,
                            params,
                            sampling_frequency),
            fail);

        if (!located)
        {
            /*
             * Only mark sync as lost if the primary ping was lost. Secondary
             * signal tracking may be too far away and thus too quiet to hear.
             * These errors should be ignored.
             */
            if (current_ping.frequency == params.primary_frequency)
            {
                sync = false;
                frequency_sync = false;
                dbprintf("Failed to find primary ping frequency.\n");
            }
            else
            {
                dbprintf("Failed to find secondary ping.\n");
            }

            continue;
        }

        tick_t offset = start_index * (CPU_CLOCK_HZ / sampling_frequency);
        const tick_t previous_ping_tick = sample_start_tick + offset;

        /*
         * Perform frequency analysis to ensure the ping frequency is
         * what is expected.
         */
        size_t last_sample = start_index + 0.002 * sampling_frequency;

        if (last_sample >= num_samples)
        {
            last_sample = num_samples - 1;
        }

        const tick_t frequency_start_tick = get_system_time();
        float primary_frequency;
        AbortIfNot(get_frequency(&samples[start_index],
                                 last_sample - start_index + 1,
                                 sampling_frequency,
                                 &primary_frequency,
                                #ifndef EMULATED
                                 &fft_socket
                                #else
                                 NULL
                                #endif
                                 ),
                fail);

        device_status.fft_frequency_hz = primary_frequency;
        device_status.fft_time_us = ticks_to_us(get_system_time() -
                                                frequency_start_tick);

        /*
         * Lock on to the proper frequency.
         */
        if (!debug_stream && !frequency_sync)
        {
            /*
             * Empty out the ring buffer queue - we will reset everything
             * within this code section.
             */
            AbortIfNot(init_ring_buffer(&future_ping_queue), fail);

            Ping ping25khz = {
                .frequency = TwentyFiveKHz,
                .time = 0
            };

            Ping ping30khz = {
                .frequency = ThirtyKHz,
                .time = 0
            };

            Ping ping35khz = {
                .frequency = ThirtyFiveKHz,
                .time = 0
            };

            Ping ping40khz = {
                .frequency = FourtyKHz,
                .time = 0
            };

            if (primary_frequency >= 22500 && primary_frequency < 27500)
            {
                switch (tracked_frequency)
                {
                    case TwentyFiveKHz:
                        ping25khz.time = previous_ping_tick + ms_to_ticks(2000);
                        ping30khz.time = previous_ping_tick + ms_to_ticks(1500);
                        ping35khz.time = previous_ping_tick + ms_to_ticks(900);
                        ping40khz.time = previous_ping_tick + ms_to_ticks(500);

                        if (params.track_40khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping40khz), fail);
                        if (params.track_35khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping35khz), fail);
                        if (params.track_30khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping30khz), fail);

                        frequency_sync = true;
                        current_ping.frequency = TwentyFiveKHz;
                        break;

                    case ThirtyKHz:
                        ping25khz.time = previous_ping_tick + ms_to_ticks(1500);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping25khz), fail);
                        break;

                    case ThirtyFiveKHz:
                        ping25khz.time = previous_ping_tick + ms_to_ticks(900);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping25khz), fail);
                        break;

                    case FourtyKHz:
                        ping25khz.time = previous_ping_tick + ms_to_ticks(500);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping25khz), fail);
                        break;

                    default:
                        AbortIfNot(0, fail);
                        break;
                }
            }
            else if (primary_frequency >= 27500 && primary_frequency < 32500)
            {
                switch (tracked_frequency)
                {
                    case TwentyFiveKHz:
                        ping30khz.time = previous_ping_tick + ms_to_ticks(500);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping30khz), fail);
                        break;

                    case ThirtyKHz:
                        ping25khz.time = previous_ping_tick + ms_to_ticks(500);
                        ping30khz.time = previous_ping_tick + ms_to_ticks(2000);
                        ping35khz.time = previous_ping_tick + ms_to_ticks(1400);
                        ping40khz.time = previous_ping_tick + ms_to_ticks(1000);

                        if (params.track_25khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping25khz), fail);
                        if (params.track_40khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping40khz), fail);
                        if (params.track_35khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping35khz), fail);

                        frequency_sync = true;
                        current_ping.frequency = ThirtyKHz;
                        break;

                    case ThirtyFiveKHz:
                        ping30khz.time = previous_ping_tick + ms_to_ticks(1400);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping30khz), fail);
                        break;

                    case FourtyKHz:
                        ping30khz.time = previous_ping_tick + ms_to_ticks(1000);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping30khz), fail);
                        break;

                    default:
                        AbortIfNot(0, fail);
                        break;
                }
            }
            else if (primary_frequency >= 32500 && primary_frequency < 37500)
            {
                switch (tracked_frequency)
                {
                    case TwentyFiveKHz:
                        ping35khz.time = previous_ping_tick + ms_to_ticks(1100);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping35khz), fail);
                        break;

                    case ThirtyKHz:
                        ping35khz.time = previous_ping_tick + ms_to_ticks(600);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping35khz), fail);
                        break;

                    case ThirtyFiveKHz:
                        ping25khz.time = previous_ping_tick + ms_to_ticks(1100);
                        ping30khz.time = previous_ping_tick + ms_to_ticks(600);
                        ping35khz.time = previous_ping_tick + ms_to_ticks(2000);
                        ping40khz.time = previous_ping_tick + ms_to_ticks(1600);

                        if (params.track_30khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping30khz), fail);
                        if (params.track_25khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping25khz), fail);
                        if (params.track_40khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping40khz), fail);

                        frequency_sync = true;
                        current_ping.frequency = ThirtyFiveKHz;
                        break;

                    case FourtyKHz:
                        ping35khz.time = previous_ping_tick + ms_to_ticks(1600);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping35khz), fail);
                        break;

                    default:
                        AbortIfNot(0, fail);
                        break;
                }
            }
            else if (primary_frequency >= 37500 && primary_frequency < 42500)
            {
                switch (tracked_frequency)
                {
                    case TwentyFiveKHz:
                        ping40khz.time = previous_ping_tick + ms_to_ticks(1500);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping40khz), fail);
                        break;

                    case ThirtyKHz:
                        ping40khz.time = previous_ping_tick + ms_to_ticks(1000);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping40khz), fail);
                        break;

                    case ThirtyFiveKHz:
                        ping40khz.time = previous_ping_tick + ms_to_ticks(400);
                        AbortIfNot(push_ring_buffer(&future_ping_queue, ping40khz), fail);
                        break;

                    case FourtyKHz:
                        ping25khz.time = previous_ping_tick + ms_to_ticks(1500);
                        ping30khz.time = previous_ping_tick + ms_to_ticks(1000);
                        ping35khz.time = previous_ping_tick + ms_to_ticks(400);
                        ping40khz.time = previous_ping_tick + ms_to_ticks(2000);

                        if (params.track_35khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping35khz), fail);
                        if (params.track_30khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping30khz), fail);
                        if (params.track_25khz)
                            AbortIfNot(push_ring_buffer(&future_ping_queue, ping25khz), fail);

                        frequency_sync = true;
                        current_ping.frequency = FourtyKHz;
                        break;

                    default:
                        AbortIfNot(0, fail);
                        break;
                }
            }
            else
            {
                /*
                 * The calculated frequency did not fall in our range of
                 * interest. It must not have been a ping.
                 */
                dbprintf("Frequency outside of range: %u\n", primary_frequency);
                sync = false;
                continue;
            }

            /*
             * If frequency synchronization was not acquired, continue searching
             * for the proper frequency and do not correlate the current signal.
             */
            if (!frequency_sync)
            {
                continue;
            }
        }
        else if (frequency_sync)
        {
            if (abs(primary_frequency - frequency_to_int(current_ping.frequency) * 1000) > 2500)
            {
                dbprintf("%4.5lf: Got unexected frequency. %2.2lf KHz, expected: %d KHz\n",
                        primary_frequency / 1000.0, frequency_to_int(current_ping.frequency));
                frequency_sync = false;
                continue;
            }
        }

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

        const tick_t correlation_start = get_system_time();

        AbortIfNot(cross_correlate(ping_start,
                                   ping_length,
                                   correlations,
                                   MAX_SAMPLES * 2,
                                   &num_correlations,
                                   &result,
                                   sampling_frequency),
            fail);

        device_status.correlation_time_us = ticks_to_us(get_system_time() -
                correlation_start);

        dbprintf("%4.5lf: Correlation results (%d KHz): %d %d %d\n",
                ticks_to_seconds(previous_ping_tick),
                frequency_to_int(current_ping.frequency),
                result.channel_delay_ns[0],
                result.channel_delay_ns[1],
                result.channel_delay_ns[2]);


        /*
         * At this point, the ping frequency should be know either through FFT
         * or through scheduling guarantees.
         */
        AbortIf(current_ping.frequency == Unknown, fail);

        #ifndef EMULATED
            /*
             * Relay the result.
             */
            AbortIfNot(send_result(&result_socket,
                                   &result,
                                   current_ping.frequency),
                fail);

            /*
             * Send the data for the correlation portion and the correlation
             * result.
             */
            AbortIfNot(send_xcorr(&xcorr_stream_socket,
                                  correlations,
                                  num_correlations),
                fail);

            AbortIfNot(send_data(&data_stream_socket, ping_start, ping_length),
                fail);
        #endif

        /*
         * Schedule the next ping for this frequency onto the queue.
         */
        current_ping.time = previous_ping_tick + ms_to_ticks(2000);
        AbortIfNot(push_ring_buffer(&future_ping_queue, current_ping),
            fail);

        device_status.ping_processing_time_us = ticks_to_us(get_system_time() - sample_start_tick);
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

    /*
     * If go fails, we need to trigger a processor reset.
     */
    while (1)
    {
        give_up();
    }
}
