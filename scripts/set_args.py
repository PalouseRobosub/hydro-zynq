import socket
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Sets an argument on the HydroZynq.')
    parser.add_argument('arg', help='The argument to set.')
    parser.add_argument('val', help='The value to set the argument to.')
    parser.add_argument('--hostname', type=str, default='192.168.0.7', help='Specifies the hostname to bind to')

    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.connect((args.hostname, 3000))

    command = '{}:{}'.format(args.arg, args.val)
    sock.send(command)

