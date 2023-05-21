import time
import socket
import math


ANY = "0.0.0.0"
SENDERPORT = 32000
MCAST_ADDR = "237.252.249.227"
MCAST_PORT = 12121
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.bind((ANY, 0))
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

data = '9,148,155,119,128,0,1,1,0,0,0,0'
sock.sendto(data.encode(),(MCAST_ADDR, MCAST_PORT))

