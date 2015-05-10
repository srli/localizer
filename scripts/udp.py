# udp.py
# TODO: move into own project
import socket as sock

def udp_socket():
    udp_sock = sock.socket(sock.AF_INET, sock.SOCK_DGRAM)
    udp_sock.setsockopt(sock.SOL_SOCKET, sock.SO_REUSEADDR, 1)
    udp_sock.setsockopt(sock.SOL_SOCKET, sock.SO_BROADCAST, 1)
    return udp_sock

# UDP to ROS reader node
class UDP_reader(object):
    def __init__(self, port, topic, data_type):
        pass

    def main_loop(self):
        pass
        
    # import socket

    # # waits until it receives data
    # UDP_IP = "255.255.255.255"
    # UDP_PORT = 60008

    # sock = socket.socket(socket.AF_INET,
    #                      socket.SOCK_DGRAM)
    # sock.bind((UDP_IP, UDP_PORT))

    # while True:
    #     data, addr = sock.recvfrom(1024) #buffer size
    #     print "received message:", data