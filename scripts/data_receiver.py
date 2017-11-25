import socket
import struct
import sys
import argparse

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


    def parse(self):
        for i in range(4, len(self.data), 8):
            self.samples.append(Sample(self.data[i:i + 8]))


def write_to_csv(packets, filename):
    low_index = 0
    high_index = 0

    for packet in packets:
        packet.parse()
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
    parser.add_argument('output', type=str, help='Specifies output file name')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.bind(('192.168.0.250', 3001))

    whole_data = []
    started = False

    while True:
        data, addr = sock.recvfrom(8 * 3000 + 4)

        packet = Packet(data);
        if packet.number is 0:
            if started:
                print('Got whole data from prev: {} long'.format(len(whole_data)))
                write_to_csv(whole_data, args.output)
                print('Written')
                sys.exit(0)
            started = True

        if started:
            whole_data.append(packet)

    #print('Got packet of len: {} (index {})'.format(len(data), series))
    #print('Packet: {}'.format(data));
