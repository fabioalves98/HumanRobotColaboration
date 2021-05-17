#!/usr/bin/env python
import socket, time, math
import rospy

from iris_cobot.msg import JointSpeed

M_PI = math.pi

HOST = "192.168.56.101" # Simulation host
# HOST = "10.1.0.2" # Real robot
PORT = 30003 # The same port as used by the server

joint_speeds = [0.1, 0, 0, 0, 0, 0]

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))


def sendJointSpeed():
    jspeed_str = str([round(js, 3) for js in joint_speeds])
    accel = '0.5'
    time = '0.05'
    speed_j_str = 'speedj(' + jspeed_str + ',' + accel + ',' + time + ')\n'

    print(speed_j_str)

    s.send(bytes(speed_j_str))


def jointSpeedSub(data):
    global joint_speeds
    joint_speeds = data.joint_speeds


def main():
    rospy.init_node('urscript', anonymous=True)

    rospy.Subscriber('joint_speeds', JointSpeed, jointSpeedSub, queue_size=1)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        start = time.time()
        sendJointSpeed()
        rate.sleep()
        end = time.time()
        print(1/(end-start))
    
    s.close()


if __name__ == '__main__':
    main()


