#!/usr/bin/env python
import rospy, math
import numpy as np
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3, WrenchStamped
from controller_manager_msgs.srv import SwitchController

from sami.gripper import Gripper
import helpers

# States
RELEASED = 0
GRIPPING = 1
GRIP_EMPTY = 2
WEIGHT_OBJ = 3
GRIP_OBJ = 4
RELEASE_OBJ = 5



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
    rospy.init_node('gripper_sm', anonymous=True)

    # global gripper
    gripper = Gripper('cr200-85', host='10.1.0.2', port=44221)

    rospy.Service('gripper_toggle', Trigger, gripperToggleServ)

    rospy.Subscriber('wrench_correct', WrenchStamped, forceCallback, queue_size=1)
    rospy.Subscriber('linear_velocity', Vector3, weightVector, queue_size=1)

    gripper_ready = GRIPPER_STANDBY 
    has_object = False

    global gripper_toggle

    next_state = RELEASED
    cur_state = RELEASED

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

        # print('\nState - %d' % cur_state)
        # print('Weight - %f' % weight)
        # print('Gripped? - %r' % gripped)
        # print('Has Object? - %r' % has_object)
        # print('Prod - %f' % prod)
        # print('Position - %f' % gripper.get_position())

        cur_state = next_state

        # 0 - Released State
        if cur_state == RELEASED:
            if gripper_toggle:
                if gripper_ready < 0:
                    gripper.grip()
                    gripper_ready = GRIPPER_STANDBY
                    next_state = GRIPPING
                gripper_toggle = False

        # 1 - Gripping State
        elif cur_state == GRIPPING:
            if gripped:
                if has_object:
                    next_state = WEIGHT_OBJ
                else:
                    next_state = GRIP_EMPTY
        
        # 2 - Gripper Empty State
        elif cur_state == GRIP_EMPTY:
            if gripper_toggle:
                if gripper_ready < 0:
                    gripper.release()
                    gripper_ready = GRIPPER_STANDBY
                    next_state = RELEASED
                gripper_toggle = False
        
        # 3 - Calibrating State
        elif cur_state == WEIGHT_OBJ:
            # TODO: Desligar o FT_2_Vel
            helpers.switchControllers(True)
            upwards_move = [-0.05, 0, 0, 0, 0, 0]
            helpers.samiMoveService(upwards_move)

            wrench_msg = rospy.wait_for_message('wrench_correct', WrenchStamped)
            weight = math.sqrt(math.pow(wrench_msg.wrench.force.x, 2) + 
                     math.pow(wrench_msg.wrench.force.y, 2) + 
                     math.pow(wrench_msg.wrench.force.z, 2))
            
            helpers.weightUpdate(weight/10)
            helpers.switchControllers(False)
            next_state = GRIP_OBJ

        # 4 - Gripper Object State
        elif cur_state == GRIP_OBJ:
            if gripper_toggle:
                helpers.switchControllers(True)
                if gripper_ready < 0:
                    gripper.release()
                    gripper_ready = GRIPPER_STANDBY
                    next_state = RELEASE_OBJ
                gripper_toggle = False

        # 5 - Release Object State
        elif cur_state == RELEASE_OBJ:
            if gripper_ready < 0:
                helpers.weightUpdate(0)
                helpers.switchControllers(False)
                next_state = RELEASED

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()