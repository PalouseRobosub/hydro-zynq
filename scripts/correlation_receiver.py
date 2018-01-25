import socket
import struct
import sys
import argparse
import numpy
import plot_data
import math
import rospy
from robosub.msg import HydrophoneDeltas

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

def pub_deltas(pub, data):
    delta_msg = HydrophoneDeltas()

    delay1 = numpy.argmax(data[:,1])
    delay2 = numpy.argmax(data[:,2])
    delay3 = numpy.argmax(data[:,3])

    delta_msg.header.stamp = rospy.Time.now()

    delta_msg.xDelta = rospy.Duration(data[delay1,0])
    delta_msg.yDelta = rospy.Duration(data[delay2,0])
    delta_msg.zDelta = rospy.Duration(data[delay3,0])

    pub.publish(delta_msg)

def calc_bearing(data):

    delay1 = numpy.argmax(data[:,1])
    delay2 = numpy.argmax(data[:,2])

    delay1 = data[delay1,0]
    delay2 = data[delay2,0]


    speed_sound_in_water = 1484.0
    hydrophone_positions = numpy.array([0.012, 0.012])

    time_deltas = numpy.array([delay1, delay2])
    d = time_deltas * speed_sound_in_water

    bearing = d / hydrophone_positions

    angle = math.atan2(bearing[1], bearing[0])

    print "bearing:"
    print "i, j: {}, {}".format(bearing[0], bearing[1])
    print "angle: {}".format(angle * 180 / math.pi)



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
    parser.add_argument('--hostname', type=str, default='192.168.0.7', help='Specifies the hostname to bind to')
    args = parser.parse_args()

    rospy.init_node("hydrozynq_interface")
    delta_pub = rospy.Publisher('hydrophones/30khz/delta', HydrophoneDeltas,
            queue_size=1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.hostname, 3003))

    channels = [0,1]
    labels = {}
    for channel in channels:
        labels[channel] = "Ch0-Ch{}".format(channel+1)

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
                calc_bearing(np_array)
                pub_deltas(pub=delta_pub, data=np_array)
# plot_data.plot_correlations(np_array, channels, labels, split=False)
                if args.output is not None:
                    write_to_csv(whole_data, args.output)
                    print('Written')

                # Reset and prepare for next batch of data.
                sock.close()
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((args.hostname, 3003))
                whole_data = []
                started = False

            else:
                started = True

        if started:
            whole_data.append(packet)

