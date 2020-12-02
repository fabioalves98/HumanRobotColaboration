#!/usr/bin/env python
import rospy, socket, sys, signal
from ur10e_control.msg import ArmCommand
from ur10e_control.srv import ControlArm

def signal_handler(sig, frame):
    print('')
    sys.exit(0)


def callControlArmService(arguments, printFeedback=True):
    rospy.wait_for_service('/cork_iris/control_arm', timeout=2.5)
    try:
        control_arm = rospy.ServiceProxy('/cork_iris/control_arm', ControlArm)
        feedback = control_arm(arguments)
        rospy.loginfo("Command sent successfully") if feedback.success else rospy.logerr("Command returned with an error")
        
        if printFeedback and feedback.output_feedback: 
            rospy.loginfo(feedback.output_feedback)

        return feedback.output_feedback
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main():
    rospy.init_node('menu', anonymous=False)
    
    signal.signal(signal.SIGINT, signal_handler)

    helpStr = callControlArmService(["help"], printFeedback=False)

    while not rospy.is_shutdown():
        print(helpStr)
        arguments = raw_input("Insert command: ").split(" ")
        if("exit" in arguments):
            break
        callControlArmService(arguments)
    
                
if __name__ == '__main__':
    main()