#!/usr/bin/env python3
import socket, time, math

M_PI = math.pi

HOST = "192.168.56.101" # The remote host
PORT = 30003 # The same port as used by the server


def socketConnection():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    # s.send ("set_digital_out(2,True)" + "\n")
    s.send(b'speedj([0.2,0,0,0,0,0], 0.5, 3)\n')
    # s.send("movej([0,1.57,-1.57,3.14,-1.57,1.57], a=1.4, v=1.05, t=0, r=0)" + "\n")

    data = s.recv(1024)

    s.close()

    print ("Received", repr(data))
    

if __name__ == '__main__':
    socketConnection()


