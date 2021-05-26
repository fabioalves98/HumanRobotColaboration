#!/usr/bin/env python

import rospy, time, signal, sys
from geometry_msgs.msg import WrenchStamped, Vector3

from sami.gripper import Gripper

GRIPPER_STANDBY = 500
INT_FORCE = 5

prev = (0, 0, 0)

gripper_ready = GRIPPER_STANDBY
tap = False
tap_counter = 0

force_integral_pub = None
gripper = None


def signal_handler(sig, frame):
    print('Ctrl + C Received')
    sys.exit(0)


def gripperToggle():
    print(time.time(), 'Double Tap')
    gripped = gripper.get_status() == 8
    if gripped:
        gripper.release()
    else:
        gripper.grip()


def wrenchCallback(data):
    x = data.wrench.force.x
    y = data.wrench.force.y
    z = data.wrench.force.z

    global prev, tap, tap_counter

    x_int = x - prev[0]
    y_int = y - prev[1]
    z_int = z - prev[2]

    prev = (x, y, z)

    # Gripper Control
    global gripper_ready
    gripper_ready -= 1


    # Tap Control
    if abs(x_int) > INT_FORCE or abs(y_int) > INT_FORCE or abs(z_int) > INT_FORCE:
        tap = True
    
    if tap == True:
        tap_counter += 1
        if tap_counter > 200:
            tap_counter = 0
            tap = False
        if tap_counter > 50 and tap_counter < 200:
            if abs(x_int) > INT_FORCE or abs(y_int) > INT_FORCE or abs(z_int) > INT_FORCE:
                if gripper_ready < 1:
                    gripperToggle()
                    gripper_ready = GRIPPER_STANDBY
                tap_counter = 0
                tap = False


    force_integral_pub.publish(Vector3(*[x_int, y_int, z_int]))


def main():
    rospy.init_node('gripper_tap', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    global gripper
    gripper = Gripper('cr200-85', host='10.1.0.2', port=44221)

    wrench_sub = rospy.Subscriber("wrench", WrenchStamped, wrenchCallback, queue_size=1)

    global force_integral_pub
    force_integral_pub = rospy.Publisher("force_integral", Vector3, queue_size=1)
        
    rospy.spin()


if __name__ == '__main__':
    main()