#include "transmission_util.h"

#include "network_stack.h"

#include "types.h"
#include "time_util.h"
#include "abort.h"
#include "udp.h"

#include <string.h>
#include "inttypes.h"

/**
 * Transmits a cross correlation result.
 *
 * @param socket The connected socket to send data over.
 * @param result The result to transmit.
 *
 * @return Success or fail.
 */
result_t send_result(udp_socket_t *socket, correlation_result_t *result)
{
    AbortIfNot(socket, fail);
    AbortIfNot(result, fail);

    char str[200];

    /*
     * Format the result into an standard message.
     */
    sprintf(str, "Result - A: %"PRId32" B: %"PRId32" C: %"PRId32,
            result->channel_delay_ns[0],
            result->channel_delay_ns[1],
            result->channel_delay_ns[2]);

    /*
     * Send the data over the socket.
     */
    AbortIfNot(send_udp(socket, str, strlen(str)), fail);

    return success;
}

/**
 * Transmits sampled data.
 *
 * @param socket The connected socket to send data over.
 * @param data The sample data to send.
 * @param count The number of samples to transmit.
 *
 * @return Success or fail.
 */
result_t send_data(udp_socket_t *socket, sample_t *data, const size_t count)
{
    #define samples_per_packet 300
    char buf[8 * samples_per_packet + 5];

    int i;
    for (i = 0; (i + samples_per_packet) < count; i += samples_per_packet)
    {
        /*
         * First, packet number in.
         */
        int packet_number = i / samples_per_packet;
        memcpy(buf, &packet_number, 4);
        for (int j = 0; j < samples_per_packet; ++j)
        {
            memcpy(&buf[4 + j * 8], data[i + j].sample, 8);
        }

        /*
         * Send the data.
         */
        AbortIfNot(send_udp(socket, buf, 8 * samples_per_packet + 4), fail);
        dispatch_network_stack();
        busywait(micros_to_ticks(50));
    }

    /*
     * If there is still data to send, send it in a smaller packet.
     */
    if (i * samples_per_packet < count)
    {
        size_t remainder = count - i * samples_per_packet;

        int packet_number = i / samples_per_packet;
        memcpy(buf, &packet_number, 4);
        for (int j = 0; j < remainder; ++j)
        {
            memcpy(&buf[4 + j * 8], data[i + j].sample, 8);
        }

        buf[8 * remainder + 4] = 0;

        /*
         * Send the data.
         */
        AbortIfNot(send_udp(socket, buf, 8 * remainder + 4), fail);
        dispatch_network_stack();
    }

    return success;
}
