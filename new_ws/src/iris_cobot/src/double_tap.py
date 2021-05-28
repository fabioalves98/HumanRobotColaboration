#!/usr/bin/env python
import rospy, time, signal, sys
from std_srvs.srv import Trigger
from geometry_msgs.msg import WrenchStamped, Vector3


DTAP_STANDBY = 500
INT_FORCE = 5

prev = (0, 0, 0)

dtap_ready = DTAP_STANDBY
tap = False
tap_counter = 0

force_integral_pub = None
gripper = None


def signal_handler(sig, frame):
    print('Ctrl + C Received')
    sys.exit(0)


def triggerToggle():
    rospy.wait_for_service('gripper_toggle')
    try:
        trigServ = rospy.ServiceProxy('gripper_toggle', Trigger)
        print(trigServ())
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


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
    global dtap_ready
    dtap_ready -= 1


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
                if dtap_ready < 1:
                    triggerToggle()
                    dtap_ready = DTAP_STANDBY
                tap_counter = 0
                tap = False


    force_integral_pub.publish(Vector3(*[x_int, y_int, z_int]))


def main():
    rospy.init_node('double_tap', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    # Visualization of the force integration
    global force_integral_pub
    force_integral_pub = rospy.Publisher("force_integral", Vector3, queue_size=1)    

    rospy.Subscriber("wrench", WrenchStamped, wrenchCallback, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()