#!/usr/bin/env python3
import socket, urx, time, math

M_PI = math.pi

HOST = "192.168.56.101" # The remote host
PORT = 30003 # The same port as used by the server


def socketConnection():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    # s.send ("set_digital_out(2,True)" + "\n")
    s.send("speedj([0.2,0,0,0,0,0], 0.5, 3)" + "\n")
    # s.send("movej([0,1.57,-1.57,3.14,-1.57,1.57], a=1.4, v=1.05, t=0, r=0)" + "\n")

    data = s.recv(1024)

    s.close()

    print ("Received", repr(data))


def urxConnection():
    rob = urx.Robot(HOST)
    rob.set_tcp((0, 0, 0, 0, 0, 0))
    rob.set_payload(2, (0, 0, 0.1))
    time.sleep(0.2)  #leave some time to robot to process the setup commands

    print('Moving Joints')
    rob.movej((0, -M_PI/2, 0, 0, 0, 0), acc=1.4, vel=1.05, wait=True)
    print('Moved Joints')

    rob.speedx('speedj', [0.2, 0, 0, 0, 0, 0], 0.5, 3)
    print('Sent Speed Command')
    while True :
        time.sleep(0.1)  #sleep first since the robot may not have processed the command yet
        if rob.is_program_running():
            print('Still Running')
            continue
        break

    rob.close()
    

if __name__ == '__main__':
    # socketConnection()
    urxConnection()


