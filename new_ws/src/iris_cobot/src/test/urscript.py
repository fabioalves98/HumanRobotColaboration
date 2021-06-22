#!/usr/bin/env python
import socket, sys, math, signal, time, struct
import rospy
from sensor_msgs.msg import JointState

from iris_cobot.msg import JointSpeed

M_PI = math.pi

# HOST = "192.168.56.101" # Simulation host
HOST = "10.1.0.2" # Real robot
PORT = 30003 # The same port as used by the server

joint_speeds = [0.2, 0, 0, 0, 0, 0]
joint_states = [0, 0, 0, 0, 0, 0]


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


def jointSpeedControl():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    jspeed_str = str([round(js, 3) for js in joint_speeds])
    
    # Simple Version
    accel = 0.5
    dt = 1
    rate = 5

    # Faster Version
    # accel = 0.5
    # dt = 0.2
    # rate = 50

    # "Correct" Version
    # accel = 0.5
    # dt = 0.002
    # rate = 500

    for i in range(rate * 2):
        start = time.time()
        speed_j_str = 'speedj(' + jspeed_str + ',' + str(accel) + ',' + str(dt) + ')\n'
        print(i, speed_j_str)
        s.send(bytes(speed_j_str))
        end = time.time()

        if (end - start) < dt:
            time.sleep(dt - (end-start))

    s.close()


def servoSpeedControl():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    dt = 0.002
    lookahead = 0.1
    gain = 300

    while True:
        start = time.time()
        wtv = s.recv(252)

        joint_pos = []

        for i in range(6):
            packet = s.recv(8).encode("hex")
            joint = struct.unpack('!d', packet.decode('hex'))[0]
            joint_pos.append(joint)
        
        wtv = s.recv(1108 - 300)

        # print(joint_pos)

        joint_pos[0] += 0.5

        jpos_str = str([round(js, 5) for js in joint_pos])
        servo_j_str = 'servoj(' + jpos_str + ',0,0,' + str(dt) + ',' + str(lookahead) + ',' + str(gain) + ')\n'
        # print(servo_j_str)
        s.send(bytes(servo_j_str))

        end = time.time()
        print(1 / (end - start))


def zeroFtSensor():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    zero_str = 'zero_ftsensor()\n'

    s.send(bytes(zero_str))

    s.close()


def jointSpeedSub(data):
    global joint_speeds
    joint_speeds = data.joint_speeds


def jointStateSub(data):
    global joint_states
    joint_states = data.position


def main():
    rospy.init_node('urscript', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    # rospy.Subscriber('joint_speeds', JointSpeed, jointSpeedSub, queue_size=1)
    # rospy.Subscriber('joint_states', JointState, jointStateSub, queue_size=1)

    # jointSpeedControl()
    # servoSpeedControl()
    zeroFtSensor()

    

if __name__ == '__main__':
    main()


