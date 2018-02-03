#!/usr/bin/python
import socket
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Displays console output from hydro-zynq board")
    parser.add_argument('--hostname', type=str, default='192.168.0.2', help='Specifies the hostname to bind to')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.hostname, 3004))

    while True:
        data = sock.recv(1024)
        print data,
