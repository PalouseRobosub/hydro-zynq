import socket
import struct
import sys
import argparse
import numpy
import plot_data

class Sample:
    def __init__(self, data):
        self.channel = [None] * 4
        [self.channel[0], self.channel[1], self.channel[2], self.channel[3]] = struct.unpack('<hhhh', data)

#        new_data = struct.pack('<HHHH', self.channel[0] << 2, self.channel[1] << 2, self.channel[2] << 2, self.channel[3] << 2)

#        [self.channel[0], self.channel[1], self.channel[2], self.channel[3]] = struct.unpack('<hhhh', new_data)


class Packet:
    def __init__(self, data):
        [self.number] = struct.unpack('<i', data[:4])
        self.samples = []
        self.data = data


    def _parse(self):
        for i in range(4, len(self.data), 8):
            self.samples.append(Sample(self.data[i:i + 8]))


def to_numpy(packets):
    for packet in packets:
        if packet.number is 0:
            samples_per_packet = len(packet.samples)

    array_list = []
    for packet in packets:
        sample_index = 0
        for sample in packet.samples:
            sample_number = sample_index + packet.number * samples_per_packet
            sample_index += 1
            sub_list = [sample_number / 5000000.0]
            for x in xrange(4):
                sub_list.append(sample.channel[x])
            array_list.append(sub_list)
    return numpy.array(array_list)


def write_to_csv(packets, filename):
    low_index = 0
    high_index = 0

    for packet in packets:
        if packet.number is 0:
            samples_per_packet = len(packet.samples)
            print('Samples per packet: {}'.format(len(packet.samples)))

    with open(filename, 'w') as f:
        f.write('Sample number, C1, C2, C3, C4\n')
        for packet in packets:
            sample_index = 0
            for sample in packet.samples:
                sample_number = sample_index + packet.number * samples_per_packet
                sample_index += 1
                f.write('{}, {}, {}, {}, {}\n'.format(sample_number, sample.channel[0], sample.channel[1], sample.channel[2], sample.channel[3]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=str, help='Specifies output file name')
    parser.add_argument('--hostname', type=str, default='192.168.0.250', help='Specifies the hostname to bind to')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.bind((args.hostname, 3001))

    whole_data = []
    started = False

    channels = [0,1,2,3]
    labels = {}
    for channel in channels:
        labels[channel] = "Ch{}".format(channel)

    plotter = plot_data.live_plot(channels, labels)

    while True:
        data, addr = sock.recvfrom(8 * 3000 + 4)

        packet = Packet(data);
        if packet.number is 0:
            if started:
                for packet in whole_data:
                    packet._parse()
                print('Got data: {} long'.format(len(whole_data)))
                np_array = to_numpy(whole_data)
                plotter.plot(np_array)
                if args.output is not None:
                    write_to_csv(whole_data, args.output)
                    print('Written')
                whole_data = []
                started = False
            else:
                started = True

        if started:
            whole_data.append(packet)

    #print('Got packet of len: {} (index {})'.format(len(data), series))
    #print('Packet: {}'.format(data));
