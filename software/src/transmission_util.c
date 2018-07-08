#include "transmission_util.h"

#include "network_stack.h"

#include "types.h"
#include "time_util.h"
#include "system.h"
#include "abort.h"
#include "udp.h"

#include <math.h>
#include <string.h>
#include "inttypes.h"

/**
 * Transforms a frequency enumeration to a number.
 *
 * @param frequency The enumeration to transform.
 *
 * @return The number, in KHz, that the frequency enumeration corresponds to.
 */
int frequency_enum_to_num(const PingFrequency frequency)
{
    switch (frequency)
    {
        case Unknown:
            return -1;

        case TwentyFiveKHz:
            return 25;

        case ThirtyKHz:
            return 30;

        case ThirtyFiveKHz:
            return 35;

        case FourtyKHz:
            return 40;

        default:
            return 0;
    }

    return 0;
}

/**
 * Transmits a cross correlation result.
 *
 * @param socket The connected socket to send data over.
 * @param result The result to transmit.
 * @param frequency The frequency of the ping.
 *
 * @return Success or fail.
 */
result_t send_result(udp_socket_t *socket,
                     correlation_result_t *result,
                     const PingFrequency frequency)
{
    AbortIfNot(socket, fail);
    AbortIfNot(result, fail);

    char str[200];

    /*
     * Format the result into an standard message.
     */
    sprintf(str, "%d KHz Result - 1: %"PRId32" 2: %"PRId32" 3: %"PRId32"\n",
            frequency_enum_to_num(frequency),
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
        busywait(micros_to_ticks(100));
    }

    /*
     * If there is still data to send, send it in a smaller packet.
     */
    if (i < count)
    {
        size_t remainder = count - i;

        int packet_number = i / samples_per_packet;
        memcpy(buf, &packet_number, 4);
        for (int j = 0; j < remainder; ++j)
        {
            memcpy(&buf[4 + j * 8], data[i + j].sample, 8);
        }

        /*
         * Send the data.
         */
        AbortIfNot(send_udp(socket, buf, 8 * remainder + 4), fail);
        dispatch_network_stack();
    }

    return success;
}

/**
 * Transmits a fourier transform over UDP.
 *
 * @param socket The socket to transmit results on.
 * @param fft The FFT data array in (real, complex) form.
 * @param sample_frequency The sampling frequency of the data.
 * @param len The number of FFt results in fft.
 *
 * @return Success or fail.
 */
result_t send_fft(udp_socket_t *socket,
                  float *fft,
                  const uint32_t sample_frequency,
                  const size_t len)
{
    AbortIfNot(socket, fail);
    AbortIfNot(fft, fail);

    #define ffts_per_packet 50
    char buf[sizeof(uint32_t) + sizeof(uint32_t) * 2 * ffts_per_packet];

    for(int i = 0; i < len / 2; i += ffts_per_packet)
    {
        int packet_number = i / ffts_per_packet;
        memcpy(buf, &packet_number, 4);

        int j = 0;
        for (j = 0; j < ffts_per_packet && i + j < len; ++j)
        {
            const uint32_t frequency = sample_frequency *
                (((float)(i + j)) / len);
            const uint32_t mag = sqrt(fft[i * 2] * fft[i * 2] +
                                      fft[i * 2 + 1] * fft[i * 2 + 1]);
            memcpy(&buf[4 + (i + j) * 8], &frequency, 4);
            memcpy(&buf[4 + (i + j) * 8 + 4], &mag, 4);
        }

        AbortIfNot(send_udp(socket, buf, 8 * j + 4), fail);
        dispatch_network_stack();
        busywait(micros_to_ticks(100));
    }

    return success;
}

result_t send_xcorr(udp_socket_t *socket, correlation_t *data, const size_t count)
{
    AbortIfNot(socket, fail);
    AbortIfNot(data, fail);

    #define correlations_per_packet 50
    char buf[16 * correlations_per_packet + 4];

    int i;
    for (i = 0; (i + correlations_per_packet) < count; i += correlations_per_packet)
    {
        /*
         * First, packet number in.
         */
        int packet_number = i / correlations_per_packet;
        memcpy(buf, &packet_number, 4);
        for (int j = 0; j < correlations_per_packet; ++j)
        {
            memcpy(&buf[4 + j * 16], &data[i + j].left_shift, 4);
            memcpy(&buf[4 + j * 16 + 4], data[i + j].result, 12);
        }

        /*
         * Send the data.
         */
        AbortIfNot(send_udp(socket, buf, 16 * correlations_per_packet + 4), fail);
        dispatch_network_stack();
        busywait(micros_to_ticks(100));
    }

    /*
     * If there is still data to send, send it in a smaller packet.
     */
    if (i < count)
    {
        size_t remainder = count - i;

        int packet_number = i / correlations_per_packet;
        memcpy(buf, &packet_number, 4);
        for (int j = 0; j < remainder; ++j)
        {
            memcpy(&buf[4 + j * 16], &data[i + j].left_shift, 4);
            memcpy(&buf[4 + j * 16 + 4], data[i + j].result, 12);
        }

        /*
         * Send the data.
         */
        AbortIfNot(send_udp(socket, buf, 16 * remainder + 4), fail);
        dispatch_network_stack();
    }

    return success;
}

result_t send_status_message(udp_socket_t *socket,
                             const DeviceStatus *device_status,
                             const HydroZynqParams *params)
{
    AbortIfNot(socket, fail);
    AbortIfNot(device_status, fail);
    AbortIfNot(device_status->firmware_rev, fail);

    char payload[1324];

    sprintf(payload, "Firmware Revision: %s\n"
                     "On Time: %4.5lf s\n"
                     "Sampling Frequency: %d Hz\n"
                     "Primary Frequency: %d\n"
                     "Fpga Temp: %d *F\n"
                     "Ping Frequency: %d Hz\n"
                     "Record + Normalize Time: %d us\n"
                     "FFT Time: %d us\n"
                     "Correlation Time: %d us\n"
                     "Ping processing Time: %d us\n"
                     "Tracking 25KHz: %d\n"
                     "Tracking 30KHz: %d\n"
                     "Tracking 35KHz: %d\n"
                     "Tracking 40KHz: %d\n"
                     "Detection Threshold: %d\n",
        device_status->firmware_rev,
        ticks_to_seconds(get_system_time()),
        (int) device_status->sampling_frequency,
        frequency_enum_to_num(params->primary_frequency),
        (int) device_status->fpga_temp_f,
        (int) device_status->fft_frequency_hz,
        (int) device_status->normalize_record_time_us,
        (int) device_status->fft_time_us,
        (int) device_status->correlation_time_us,
        (int) device_status->ping_processing_time_us,
        (int) params->track_25khz,
        (int) params->track_30khz,
        (int) params->track_35khz,
        (int) params->track_40khz,
        (int) params->ping_threshold);

    #ifndef EMULATED
        AbortIfNot(send_udp(socket, payload, strlen(payload)), fail);
        dispatch_network_stack();
    #endif

    return success;
}
