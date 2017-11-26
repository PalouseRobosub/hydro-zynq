#!/usr/bin/python

import socket
import struct
import argparse
#from data_receiver import Packet, Sample

"""
Packet Format:
PACKET_NUMBER (int32) | <samples>

where each sample is:
| ch0 (uint16) | ch1 (uint16) | ch2 (uint16) | ch3 (uint16) |

"""

def make_packet(packet_number):
    raw_data = []

    raw_data.append(struct.pack('<i', packet_number))
    raw_data.append(struct.pack('<hhhh', 1, 2, 3, 4))
    raw_data.append(struct.pack('<hhhh', 5, 6, 7, 8))

    return "".join(raw_data)

if __name__ == "__main__":
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    addr = ('localhost', 3001)

    for i in xrange (5):
        sock.sendto(make_packet(i), addr)

    # send a 0 packet number to cause receiver to quit
    sock.sendto(struct.pack('<ihhhh', 0, 0, 0, 0, 0), addr)
