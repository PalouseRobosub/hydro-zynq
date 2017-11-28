import socket
import struct
import sys
import argparse
import numpy
import plot_data

class Sample:
    def __init__(self, data):
        self.channel = [None] * 3
        [self.lshift, self.channel[0], self.channel[1], self.channel[2]] = struct.unpack('<iiii', data)


class Packet:
    def __init__(self, data):
        [self.number] = struct.unpack('<i', data[:4])
        self.samples = []
        self.data = data


    def _parse(self):
        for i in range(4, len(self.data), 16):
            self.samples.append(Sample(self.data[i:i + 16]))


def to_numpy(packets):
    array_list = []
    for packet in packets:
        for sample in packet.samples:
            sub_list = [sample.lshift / 5000000.0]
            for x in xrange(3):
                sub_list.append(sample.channel[x])
            array_list.append(sub_list)
    return numpy.array(array_list)


def write_to_csv(packets, filename):
    low_index = 0
    high_index = 0

    with open(filename, 'w') as f:
        f.write('Left Shift, C1, C2, C3\n')
        for packet in packets:
            for sample in packet.samples:
                f.write('{}, {}, {}, {}\n'.format(sample.lshift, sample.channel[0], sample.channel[1], sample.channel[2]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=str, help='Specifies output file name')
    parser.add_argument('--hostname', type=str, default='192.168.0.250', help='Specifies the hostname to bind to')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.bind((args.hostname, 3003))

    whole_data = []
    started = False

    while True:
        data, addr = sock.recvfrom(16 * 300 + 4)

        packet = Packet(data);
        if packet.number is 0:
            if started:
                for packet in whole_data:
                    packet._parse()
                print('Got data: {} long'.format(len(whole_data)))
                np_array = to_numpy(whole_data)
                plot_data.plot(np_array, [0])
                if args.output is not None:
                    write_to_csv(whole_data, args.output)
                    print('Written')
                sys.exit(0)

            started = True

        if started:
            whole_data.append(packet)

