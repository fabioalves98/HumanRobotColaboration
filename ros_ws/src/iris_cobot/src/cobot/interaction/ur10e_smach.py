#!/usr/bin/env python
import rospy, time, math
import smach, smach_ros
from geometry_msgs.msg import WrenchStamped

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


# define state FreeDrive
class Freedrive(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['dTapX-', 'dTapX+'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Freedrive')
        
        while not rospy.is_shutdown():
            action = self.getAction()
            if action.type == 'double' and action.component == 0:
                return 'dTapX+' if action.direction else 'dTapX-'


# define state Gripping
class Gripping(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['object', 'no_object'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Gripping')

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

        helpers.switchControllers(True)
        upwards_move = [0, 0, 0.05, 0, 0, 0]
        helpers.samiMoveWorldService(upwards_move)

        wrench_msg = rospy.wait_for_message('wrench_correct', WrenchStamped)
        weight = math.sqrt(math.pow(wrench_msg.wrench.force.x, 2) + 
                math.pow(wrench_msg.wrench.force.y, 2) + 
                math.pow(wrench_msg.wrench.force.z, 2))

        weight /= 1.2

        print('Calculated Weight - ', weight)
        
        helpers.weightUpdate(weight/10)
        time.sleep(1)
        helpers.switchControllers(False)
        
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

        helpers.switchControllers(True)
        helpers.samiReleaseService()
        helpers.weightUpdate(0)
        time.sleep(1)
        helpers.switchControllers(False)
        
        while not rospy.is_shutdown():
            status = self.getGripperStatus()
            if not status.gripped:
                return 'released'


# define state Fine Control
class FineControl(UR10eState):
    def __init__(self):
        UR10eState.__init__(self, outcomes=['dTapX-', 'dTapX+'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Fine Control')
        
        while not rospy.is_shutdown():
            action = self.getAction()
            if action.type == 'double' and action.component == 0:
                return 'dTapX+' if action.direction else 'dTapX-'



def main():
    rospy.init_node('ur10e_smach')

    # Create the top level UR10e state machine
    sm_ur10e = smach.StateMachine(outcomes=['exit_state'])
    
    with sm_ur10e:
        smach.StateMachine.add('FreeDrive', Freedrive(),
                            transitions={'dTapX-':'FineControl',
                                         'dTapX+':'Gripper'})

        # Create the sub Gripper state machine
        sm_gripper = smach.StateMachine(outcomes=['exit_state'])

        with sm_gripper:
            smach.StateMachine.add('Gripping', Gripping(),
                                transitions={'object':'GripObject',
                                             'no_object':'GripEmpty'})
            smach.StateMachine.add('GripObject', GripObject(),
                                transitions={'dTapY':'Releasing'})
            smach.StateMachine.add('GripEmpty', GripEmpty(),
                                transitions={'dTapY':'Releasing'})
            smach.StateMachine.add('Releasing', Releasing(),
                                transitions={'released':'exit_state'})


        smach.StateMachine.add('Gripper', sm_gripper, 
                            transitions={'exit_state':'FreeDrive'})
        
        smach.StateMachine.add('FineControl', FineControl(),
                            transitions={'dTapX-':'Gripper',
                                         'dTapX+':'FreeDrive'})
    
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