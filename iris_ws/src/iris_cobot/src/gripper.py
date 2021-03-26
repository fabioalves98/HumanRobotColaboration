#!/usr/bin/env python
from os import WIFCONTINUED
import rospy, time
import math
from geometry_msgs.msg import WrenchStamped

from sami.gripper import Gripper
import helpers

GRIPPER_STANDBY = 1000

gripper = None
gripper_ready = GRIPPER_STANDBY
has_object = False

def gripperControl(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    weight = math.sqrt(math.pow(f_x, 2) + math.pow(f_y, 2) + math.pow(f_z, 2))

    global gripper_ready
    gripper_ready -= 1
    gripped = gripper.get_status() == 8
    has_object = gripped and gripper.get_position() > 27

    print('\nWeight %f' % weight)
    print('Gripped? - %r' % gripped)
    print('Has Object? - %r' % has_object)

    # Example of gripper control through weight
    # if gripper_ready < 0:
    #     if weight > 10:
    #         if gripped:
    #             gripper.release()
    #             gripper_ready = GRIPPER_STANDBY
    #         else:
    #             gripper.grip()
    #             gripper_ready = GRIPPER_STANDBY

    # Example of gripper control throgh specific force on each axis
    if not has_object:
        if gripper_ready < 0:
            if abs(f_x) > 10:
                if not gripped:
                    gripper.grip()
                    print('Gripping')
                    gripper_ready = GRIPPER_STANDBY
            elif abs(f_y) > 10:
                if gripped:
                    gripper.release()
                    print('Releasing')
                    gripper_ready = GRIPPER_STANDBY

    # Example of gripper contorl when handling objects
    else:
        if gripper_ready < 0:
            if abs(weight) < 1.5 or abs(f_z) > 10:
                    gripper.release()
                    print('Releasing Object')
                    gripper_ready = GRIPPER_STANDBY


def main():
    rospy.init_node('gripper', anonymous=True)

    global gripper 
    gripper = Gripper('cr200-85', host='10.1.0.2', port=44221)

    helpers.reset_ft_sensor()

    wrench_sub = rospy.Subscriber('wrench_filtered', WrenchStamped, gripperControl)

    rospy.spin()


if __name__ == '__main__':
    main()