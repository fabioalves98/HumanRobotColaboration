#!/usr/bin/env python
import rospy, math
import numpy as np
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3, WrenchStamped

from sami.gripper import Gripper
import helpers

GRIPPER_STANDBY = 500

# gripper_ready = GRIPPER_STANDBY
# gripper = None
# gripped = False

gripper_toggle = False

weight_vector = [0, 0, 0]
force = [0, 0, 0]


def gripperToggleServ(data):
    global gripper_toggle
    gripper_toggle = True

    return [True ,'Gripper Toggled']


def weightVector(data):
    global weight_vector
    weight_vector = helpers.pointToList(data)


def forceCallback(data):
    global force
    force[0] = data.wrench.force.x
    force[1] = data.wrench.force.y
    force[2] = data.wrench.force.z


def main():
    rospy.init_node('gripper', anonymous=True)

    # global gripper
    gripper = Gripper('cr200-85', host='10.1.0.2', port=44221)

    rospy.Service('gripper_toggle', Trigger, gripperToggleServ)

    rospy.Subscriber('wrench_correct', WrenchStamped, forceCallback, queue_size=1)
    rospy.Subscriber('linear_velocity', Vector3, weightVector, queue_size=1)

    gripper_ready = GRIPPER_STANDBY 
    has_object = False

    global gripper_toggle

    rate = rospy.Rate(500)

    while not rospy.is_shutdown():
        weight = math.sqrt(math.pow(force[0], 2) + math.pow(force[1], 2) + math.pow(force[2], 2))

        # Gripper Controls
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

        if gripper_toggle:
            if gripper_ready < 0:
                if gripped:
                    gripper.release()
                else:
                    gripper.grip()
                gripper_ready = GRIPPER_STANDBY
            gripper_toggle = False
        
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()