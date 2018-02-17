import socket
import struct
import sys
import argparse
import os
import numpy
import plot_data
import zipfile
import progressbar

class Sample:
    def __init__(self, data):
        self.channel = [None] * 4
        [self.channel[0], self.channel[1], self.channel[2], self.channel[3]] = struct.unpack('<hhhh', data)


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

    print 'Searching for start packet...'
    for packet in packets:
        if packet.number is 0:
            samples_per_packet = len(packet.samples)
            print('Samples per packet: {}'.format(len(packet.samples)))
            break

    with open(filename, 'w') as f:
        f.write('Sample number, C1, C2, C3, C4\n')
        print 'Starting file write.'
        bar = progressbar.ProgressBar(max_value=len(packets))
        for i, packet in enumerate(packets):
            bar.update(i)
            sample_index = 0
            for sample in packet.samples:
                sample_number = sample_index + packet.number * samples_per_packet
                sample_index += 1
                f.write('{}, {}, {}, {}, {}\n'.format(sample_number, sample.channel[0], sample.channel[1], sample.channel[2], sample.channel[3]))
        bar.update(len(packets))

    archive = zipfile.ZipFile('{}.zip'.format(os.path.splitext(filename)[0]), 'w', zipfile.ZIP_DEFLATED)
    archive.write(filename)
    archive.close()
    os.remove(filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=str, help='Specifies output file name')
    parser.add_argument('--hostname', type=str, default='192.168.0.2', help='Specifies the hostname to bind to')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.hostname, 3001))

    whole_data = []
    started = False

    channels = [0,1,2,3]
    labels = {}
    for channel in channels:
        labels[channel] = "Ch{}".format(channel)


    bar = progressbar.ProgressBar(max_value=progressbar.UnknownLength)
    while True:
        data, addr = sock.recvfrom(8 * 3000 + 4)

        packet = Packet(data);

        if started and packet.number is not 0:
            bar.update(packet.number)

        if packet.number is 0:
            if started:
                print 'Starting parse.'
                parse_bar = progressbar.ProgressBar(max_value=len(whole_data))
                for i, packet in enumerate(whole_data):
                    parse_bar.update(i)
                    packet._parse()
                parse_bar.update(len(whole_data))
                np_array = to_numpy(whole_data)
                if args.output is not None:
                    write_to_csv(whole_data, args.output)
                    print('CSV data written to {}.zip'.format(os.path.splitext(args.output)[0]))
                    sys.exit(0)

                plot_data.plot_samples(np_array, channels, labels, split=False)

                # Reset and prepare for next batch of data.
                sock.close()
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((args.hostname, 3001))
                whole_data = []
                started = False

            else:
                print 'Starting log'
                started = True

        if started:
            whole_data.append(packet)

    #print('Got packet of len: {} (index {})'.format(len(data), series))
    #print('Packet: {}'.format(data));
