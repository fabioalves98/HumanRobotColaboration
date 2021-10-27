#!/usr/bin/env python
import time, math
import rospy
import smach, smach_ros
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
from tf2_geometry_msgs import do_transform_vector3

import cobot.helpers as helpers

from iris_cobot.msg import TapAction
from iris_sami.msg import GripperInfo


class UR10eState(smach.State):
    def __init__(self, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)


    def getGripperStatus(self):
        gripper_status = rospy.wait_for_message("iris_sami/gripper_status", GripperInfo)
        return gripper_status


    def getAction(self):
        tap_action = rospy.wait_for_message("tap_action", TapAction)
        return tap_action


# define state HandGuiding
class HandGuiding(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['dTapX-', 'dTapX+', 'dTapY'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Hand Guiding')

        helpers.cobot_reset_ft_sensor()
        helpers.hgControl('play')
        helpers.switchControllers('velocity')
        
        while not rospy.is_shutdown():
            action = self.getAction()
            if action.type == 'double' and action.component == 0:
                return 'dTapX+' if action.direction else 'dTapX-'
            if action.type == 'double' and action.component == 1:
                return 'dTapY'


# define state Gripping
class Gripping(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['object', 'no_object'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Gripping')

        helpers.switchControllers("position")

        helpers.samiGripService()
        
        while not rospy.is_shutdown():
            status = self.getGripperStatus()
            if status.gripped:
                if status.has_object:
                    return 'object'
                else:
                    return 'no_object'


# define state GripObject
class GripObject(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['dTapY'])


    def execute(self, userdata):
        rospy.loginfo('Executing state GripObject')

        upwards_move = [0, 0, 0.05, 0, 0, 0]
        helpers.samiMoveWorldService(upwards_move)
        time.sleep(0.5)

        wrench_msg = rospy.wait_for_message('wrench_correct', WrenchStamped)
        weight = math.sqrt(math.pow(wrench_msg.wrench.force.x, 2) + 
                math.pow(wrench_msg.wrench.force.y, 2) + 
                math.pow(wrench_msg.wrench.force.z, 2))

        weight /= 1.2

        print('Calculated Weight - ', weight)
        
        helpers.weightUpdate(weight/10)
        time.sleep(0.5)
        helpers.switchControllers("velocity")
        
        while not rospy.is_shutdown():
            action = self.getAction()
            if action.type == 'double' and action.component == 1:
                return 'dTapY'


# define state GripEmpty
class GripEmpty(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['dTapY'])


    def execute(self, userdata):
        rospy.loginfo('Executing state GripEmpty')
        
        while not rospy.is_shutdown():
            action = self.getAction()
            if action.type == 'double' and action.component == 1:
                return 'dTapY'


# define state Releasing
class Releasing(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['released'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Releasing')

        helpers.switchControllers("position")
        helpers.samiReleaseService()
        helpers.weightUpdate(0)
        time.sleep(1)
        helpers.cobot_reset_ft_sensor()
        helpers.switchControllers("velocity")
        
        while not rospy.is_shutdown():
            status = self.getGripperStatus()
            if not status.gripped:
                return 'released'


# define state Picking
class Picking(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['OK'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Picking')
        
        helpers.switchControllers('position')
        helpers.samiAliasService('colab_pick')
        helpers.samiMoveService([0.05, 0, 0, 0, 0, 0])
        
        return 'OK'


# deifne state Grip
class Grip(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['OK'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Grip')

        helpers.samiGripService()

        while not rospy.is_shutdown():
            status = self.getGripperStatus()
            if status.gripped:
                if status.has_object:
                    return 'OK'
                else:
                    continue


# define state Picking
class Delivering(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['OK'])        

    def execute(self, userdata):
        rospy.loginfo('Executing state Picking')

        helpers.samiAliasService('colab_main')

        weight = rospy.wait_for_message("wrench_correct", WrenchStamped)

        print(weight)

        while not rospy.is_shutdown():
            wrench_velocity = rospy.wait_for_message('wrench_velocity', WrenchStamped)
            
            print(wrench_velocity.wrench.force.z)

            if wrench_velocity.wrench.force.z > 0:
                helpers.samiReleaseService()
                time.sleep(1)
                helpers.cobot_reset_ft_sensor()
                return 'OK'


# define state Industrial
class Industrial(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['dTap'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Industrial')

        # Disable Wrench to Vel
        helpers.hgControl('pause')
        time.sleep(0.1)
        helpers.hgControl('stop')
        time.sleep(0.1)
        # Enable PF Controller
        helpers.pfControl('play')
        
        while not rospy.is_shutdown():
            action = self.getAction()     
            if action.type == 'double' and action.component == 1:      
                helpers.pfControl('pause')
                time.sleep(0.1)
                helpers.pfControl('stop')
                time.sleep(0.1)
                return 'dTap'


def main():
    rospy.init_node('ur10e_smach')

    # Create the top level UR10e state machine
    sm_ur10e = smach.StateMachine(outcomes=['exit_state'])
    
    with sm_ur10e:
        smach.StateMachine.add('Hand Guiding', HandGuiding(),
                            transitions={'dTapX-':'Pick & Deliver',
                                         'dTapX+':'Object Manipulation',
                                         'dTapY':'Industrial Task'})

        # Create the sub Gripper state machine
        sm_object = smach.StateMachine(outcomes=['exit_state'])

        with sm_object:
            smach.StateMachine.add('Gripping', Gripping(),
                                transitions={'object':'GripObject',
                                             'no_object':'GripEmpty'})
            smach.StateMachine.add('GripObject', GripObject(),
                                transitions={'dTapY':'Releasing'})
            smach.StateMachine.add('GripEmpty', GripEmpty(),
                                transitions={'dTapY':'Releasing'})
            smach.StateMachine.add('Releasing', Releasing(),
                                transitions={'released':'exit_state'})
        
        smach.StateMachine.add('Object Manipulation', sm_object, 
                            transitions={'exit_state':'Hand Guiding'})
        
        # Create the sub Pick and Deliver state machine
        sm_pick_deliver = smach.StateMachine(outcomes=['exit_state'])

        with sm_pick_deliver:
            smach.StateMachine.add('Picking', Picking(),
                                transitions={'OK':'Grip'})
            smach.StateMachine.add('Grip', Grip(),
                                transitions={'OK':'Delivering'})
            smach.StateMachine.add('Delivering', Delivering(),
                                transitions={'OK':'exit_state'})
            

        smach.StateMachine.add('Pick & Deliver', sm_pick_deliver, 
                            transitions={'exit_state':'Hand Guiding'})
    
        
        smach.StateMachine.add('Industrial Task', Industrial(),
                            transitions={'dTap':'Hand Guiding'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('ur10e_smach_server', sm_ur10e, '/SM_UR10e')
    sis.start()

    # Execute SMACH plan
    outcome = sm_ur10e.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()