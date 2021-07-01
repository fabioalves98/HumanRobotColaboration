#!/usr/bin/env python
import rospy, time, signal, sys
from std_srvs.srv import Trigger
from geometry_msgs.msg import WrenchStamped, Vector3

from iris_cobot.msg import TapAction


DTAP_STANDBY = 500
INT_FORCE = 5

f_prev = (0, 0, 0)
t_prev = (0, 0, 0)

dtap_ready = DTAP_STANDBY
s_tap = False
s_tap_counter = 0
tap = False
tap_counter = 0

tap_action_pub = None
force_integral_pub = None
torque_integral_pub = None
gripper = None


def signal_handler(sig, frame):
    print('Ctrl + C Received')
    sys.exit(0)


def triggerToggle():
    rospy.wait_for_service('gripper_toggle')
    try:
        trigServ = rospy.ServiceProxy('gripper_toggle', Trigger)
        trigServ()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def wrenchCallback(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    t_x = data.wrench.torque.x
    t_y = data.wrench.torque.y
    t_z = data.wrench.torque.z

    global f_prev, t_prev, tap, tap_counter, s_tap, s_tap_counter

    f_x_int = f_x - f_prev[0]
    f_y_int = f_y - f_prev[1]
    f_z_int = f_z - f_prev[2]

    f_prev = (f_x, f_y, f_z)

    t_x_int = t_x - t_prev[0]
    t_y_int = t_y - t_prev[1]
    t_z_int = t_z - t_prev[2]

    t_prev = (t_x, t_y, t_z)

    # Gripper Control
    global dtap_ready
    dtap_ready -= 1

    # Tap Control
    int_forces = [f_x_int, f_y_int, f_z_int]
    for f in int_forces:
        if abs(f) > INT_FORCE and not s_tap:
            s_tap = True
            component = int_forces.index(f)
            direction = f > 0
            print("Single - %s - %s" % (component, 'Positive' if direction else 'Negative'))

            tap_action_pub.publish(TapAction(type='single', component=component, direction=direction))

        if s_tap == True:
            s_tap_counter +=1
            
            if s_tap_counter > 200*3:
                s_tap_counter = 0
                s_tap = False
            
            if s_tap_counter > 50*3 and s_tap_counter < 200*3:
                for f in int_forces:
                    if abs(f) > INT_FORCE:
                        if dtap_ready < 1:
                            component = int_forces.index(f)
                            direction = f > 0
                            print("Double - %s - %s" % (component, 'Positive' if direction else 'Negative'))
                            # triggerToggle()
                            tap_action_pub.publish(TapAction(type='double', component=component, direction=direction))

                            dtap_ready = DTAP_STANDBY


    force_integral_pub.publish(Vector3(*[f_x_int, f_y_int, f_z_int]))
    torque_integral_pub.publish(Vector3(*[t_x_int, t_y_int, t_z_int]))



def main():
    rospy.init_node('double_tap', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    # Visualization of the force integration
    global force_integral_pub, torque_integral_pub
    force_integral_pub = rospy.Publisher("force_integral", Vector3, queue_size=1)    
    torque_integral_pub = rospy.Publisher("torque_integral", Vector3, queue_size=1) 

    global tap_action_pub
    tap_action_pub = rospy.Publisher("tap_action", TapAction, queue_size=1)

    rospy.Subscriber("wrench", WrenchStamped, wrenchCallback, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()