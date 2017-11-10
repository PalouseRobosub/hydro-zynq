import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind(('192.168.0.250', 3000))

while True:
    data, addr = sock.recvfrom(1024)
    print data
