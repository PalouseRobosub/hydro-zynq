#!/usr/bin/python

import argparse
import rospy
import socket
import std_srvs


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--hostname', type=str, default='192.168.0.2', help='Specifies the hostname to bind to')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.hostname, 3005))

    rospy.init_node('thruster_silencer')

    control_shutdown_srv = rospy.ServiceProxy('control/silence', std_srvs.SetBool)

    while not rospy.is_shutdown():
        data = sock.receive(1024)
        recv_time = rospy.get_time()
        if len(data) != 8:
            rospy.logwarn('Received invalid packet size.')
            continue

        when, duration = struct.unpack(data, '<ii')

        while rospy.get_time() < recv_time + (when / 1000.0):
            continue

        control_shutdown_srv(True)

        shutdown_time = rospy.get_time()
        while rospy.get_time() < shutdown_time + (duration / 1000.0):
            continue

        control_shutdown_srv(False)
