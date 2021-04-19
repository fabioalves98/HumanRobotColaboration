#!/usr/bin/env python
import rospy, math
import numpy as np
from geometry_msgs.msg import Vector3, WrenchStamped

from sami.gripper import Gripper
import helpers

GRIPPER_STANDBY = 500

gripper = None
gripper_ready = GRIPPER_STANDBY
has_object = False
weight_vector = [0, 0, 0]

def weightVector(data):
    global weight_vector
    weight_vector = helpers.pointToList(data)

def gripperControl(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    weight = math.sqrt(math.pow(f_x, 2) + math.pow(f_y, 2) + math.pow(f_z, 2))

    # Gripper Controls
    global gripper_ready
    gripper_ready -= 1
    gripped = gripper.get_status() == 8
    has_object = gripped and gripper.get_position() > 27

    # Weight direction analysis
    v_g = [0, 0, -1]
    prod = np.inner(v_g, weight_vector)

    print('\nWeight %f' % weight)
    print('Gripped? - %r' % gripped)
    print('Has Object? - %r' % has_object)
    print('Prod - %f' % prod)

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
            if abs(weight) < 2 or abs(f_z) > 10 or prod < 0:
                    gripper.release()
                    print('Releasing Object')
                    gripper_ready = GRIPPER_STANDBY


def main():
    rospy.init_node('gripper', anonymous=True)

    global gripper
    gripper = Gripper('cr200-85', host='10.1.0.2', port=44221)

    rospy.Subscriber('wrench_correct', WrenchStamped, gripperControl, queue_size=1)
    rospy.Subscriber('weight_vector', Vector3, weightVector, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()