import socket
import struct
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('dstname', type=str, help='Specifies the output host to retransmit to')
    parser.add_argument('--hostname', type=str, default='192.168.0.2', help='Specifies the hostname to bind to')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.hostname, 3001))

    rebroadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rebroadcast_socket.connect((args.dstname, 3001))


    while True:
        data, addr = sock.recvfrom(8 * 3000 + 4)
        rebroadcast_socket.send(data)

