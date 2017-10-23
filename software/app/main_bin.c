/**
 * HydroZynq Data acquisition firmware.
 *
 * @author Ryan Summers
 * @date 10/17/2017
 */

#include "types.h"
#include "abort.h"
#include "system_params.h"
#include "system.h"
#include "fifo_stream.h"
#include "time_util.h"
#include "lwip/udp.h"
#include "lwip/ip.h"
#include "network_stack.h"
#include "udp.h"
#include "sample_util.h"
#include "correlation_util.h"
#include "transmission_util.h"

/**
 * The stream FIFO used for reading samples.
 */
fifo_stream_t sample_data_stream;

/**
 * The array of current samples.
 */
sample_t samples[MAX_SAMPLES];

bool debug_stream = false;

void receive_command(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, uint16_t port)
{
    debug_stream = true;
    pbuf_free(p);
}

/**
 * Main entry point into the application.
 *
 * @return Zero upon success.
 */
int main()
{
    /*
     * Initialize the system.
     */
    AbortIfNot(init_system(), fail);

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
    uprintf("Network stack initialized\n");

    /*
     * Configure the ADC.
     */
    //TODO Configure the ADC.

    /*
     * Set up the AXI FIFO stream.
     */
    AbortIfNot(init_fifo_stream(&sample_data_stream, STREAM_FIFO_BASE_ADDRESS), fail);

    /*
     * Bind the command port, data stream port, and the result output port.
     */
    udp_socket_t command_socket, data_stream_socket, result_socket;

    struct ip_addr dest_ip;
    IP4_ADDR(&dest_ip, 192, 168, 0, 2);

    AbortIfNot(init_udp(&command_socket), fail);
    AbortIfNot(bind_udp(&command_socket, IP_ADDR_ANY, COMMAND_SOCKET_PORT, receive_command), fail);

    AbortIfNot(init_udp(&data_stream_socket), fail);
    AbortIfNot(connect_udp(&data_stream_socket, &dest_ip, DATA_STREAM_PORT), fail);

    AbortIfNot(init_udp(&result_socket), fail);
    AbortIfNot(connect_udp(&result_socket, &dest_ip, RESULT_PORT), fail);

    uprintf("System initialization complete. Start time: %d ms\n",
            ticks_to_ms(get_system_time()));

    bool sync = false;
    tick_t previous_ping_tick = 0;
    while (1)
    {
        /*
         * Push received network traffic into the network stack.
         */
        dispatch_network_stack();

        /*
         * Find sync for the start of a ping if we are not debugging.
         */
        if (!sync && !debug_stream)
        {
            while (!previous_ping_tick)
            {
                AbortIfNot(acquire_sync(&sample_data_stream,
                                        samples,
                                        MAX_SAMPLES,
                                        ms_to_ticks(2000),
                                        &previous_ping_tick), fail);
            }
        }

        /*
         * Fast forward the previous ping tick until the most likely time
         * of the most recent ping.
         */
        while (get_system_time() > (previous_ping_tick + ms_to_ticks(1900)))
        {
            previous_ping_tick += ms_to_ticks(2000);
        }

        /*
         * Record the ping. When debugging, over 2 seconds of data should be
         * recovered.
         */
        uint32_t sample_duration_ms = (debug_stream)? 2100 : 200;

        size_t num_samples;
        AbortIfNot(record(&sample_data_stream, samples, MAX_SAMPLES, &num_samples,
                    ms_to_ticks(sample_duration_ms)), fail);

        /*
         * Filter the received signal.
         */
        filter_coefficients_t coefficients;
        AbortIfNot(filter(samples, num_samples, &coefficients), fail);

        /*
         * Truncate the data around the ping.
         */
        size_t start_index, end_index;
        bool located;
        AbortIfNot(truncate(samples, num_samples, &start_index, &end_index, &located), fail);
        if (!located)
        {
            uprintf("Failed to find the ping.\n");
        }

        /*
         * Locate the ping samples.
         */
        const sample_t *ping_start = &samples[start_index];
        const size_t ping_length = end_index - start_index;

        /*
         * Perform the correlation on the data.
         */
        correlation_result_t result;
        AbortIfNot(cross_correlate(ping_start, ping_length, &result), fail);

        /*
         * Relay the result.
         */
        AbortIfNot(send_result(&result_socket, &result), fail);

        /*
         * If debug streams are enabled, transmit the data used for the
         * correlation.
         */
        if (debug_stream)
        {
            AbortIfNot(send_data(&data_stream_socket, samples, num_samples), fail);
            debug_stream = false;
        }
    }
}
