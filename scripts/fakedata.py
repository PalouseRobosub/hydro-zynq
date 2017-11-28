#!/usr/bin/python

import socket
import struct
import argparse
import time
#from data_receiver import Packet, Sample

"""
Packet Format:
PACKET_NUMBER (int32) | <samples>

where each sample is:
| ch0 (uint16) | ch1 (uint16) | ch2 (uint16) | ch3 (uint16) |

"""

def make_packet(packet_number, start=0):
    raw_data = []

    raw_data.append(struct.pack('<i', packet_number))
    raw_data.append(struct.pack('<hhhh', start+1, start+2, start+3, start+4))
    raw_data.append(struct.pack('<hhhh', start+5, start+6, start+7, start+8))

    return "".join(raw_data)

if __name__ == "__main__":
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    addr = ('localhost', 3001)
    start = 0

    while 1:
        for i in xrange (5):
            sock.sendto(make_packet(i, start=start), addr)
        start += 1
        time.sleep(2)

    # send a 0 packet number to cause receiver to quit
    #sock.sendto(struct.pack('<ihhhh', 0, 0, 0, 0, 0), addr)
