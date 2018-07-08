#!/usr/bin/python

import argparse
import rospy
import re
import socket
from robosub.msg import HydrophoneDeltas


class DeltaPacket:
    def __init__(self, data):
        matches = re.search(r'(-?\d+) KHz.*1: (-?\d+) 2: (-?\d+) 3: (-?\d+).*', data)
        if not matches:
            raise Exception('Invalid result string')

        if len(matches.groups()) != 3:
            raise Exception('Valid number of groups')

        self.frequency = int(matches.group(1))
        self.x = int(matches.group(2))
        self.y = int(matches.group(3))
        self.z = int(matches.group(4))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--hostname', type=str, default='192.168.0.2', help='Specifies the hostname to bind to')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.hostname, 3002))

    rospy.init_node('hydrophone_interface')

    delta_pub_25khz = rospy.Publisher('hydrophones/25khz/delta',
            HydrophoneDeltas, queue_size=1)

    delta_pub_30khz = rospy.Publisher('hydrophones/30khz/delta',
            HydrophoneDeltas, queue_size=1)

    delta_pub_35khz = rospy.Publisher('hydrophones/35khz/delta',
            HydrophoneDeltas, queue_size=1)

    delta_pub_40khz = rospy.Publisher('hydrophones/40khz/delta',
            HydrophoneDeltas, queue_size=1)

    while not rospy.is_shutdown():
        data = sock.recv(1024)

        try:
            deltas = DeltaPacket(data)
        except Exception as e:
            rospy.logwarn('Received invalid HydroZynq datagram: {}'.format(e))
            continue

        msg = HydrophoneDeltas()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'hydrophone_array'
        msg.xDelta = rospy.Duration(deltas.x)
        msg.yDelta = rospy.Duration(deltas.y)
        msg.zDelta = rospy.Duration(deltas.z)

        if deltas.frequency == 25:
            delta_pub_25khz.publish(msg)
        elif deltas.frequency == 30:
            delta_pub_30khz.publish(msg)
        elif deltas.frequency == 35:
            delta_pub_35khz.publish(msg)
        elif deltas.frequency == 40:
            delta_pub_40khz.publish(msg)
        else:
            rospy.logwarn('Got delta for unknown frequency: {}'.format(deltas.frequency))
