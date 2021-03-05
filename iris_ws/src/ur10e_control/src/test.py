#!/usr/bin/env python
import rospy
import time, signal, sys
from math import pi, sin, cos, acos, sqrt, radians
from ur10e_control.srv import ControlArm
from std_srvs.srv import Trigger
from ArmControl import ArmControl


def signal_handler(sig, frame):
    print('')
    sys.exit(0)


def axisAngleToQuaterion():
    # Axis Angle to Quaternion
    ax = 0
    ay = 1
    az = 0
    angle = pi/2

    qx = ax * sin(angle/2)
    qy = ay * sin(angle/2)
    qz = az * sin(angle/2)
    qw = cos(angle/2)


def quaternionToAxisAngle():
    # Quaternion to Axis Angle
    pose = arm.getPose()
    print(pose.orientation)
    
    ori = pose.orientation

    angle = 2 * acos(ori.w)
    x = ori.x / sqrt(1-ori.w*ori.w)
    y = ori.y / sqrt(1-ori.w*ori.w)
    z = ori.z / sqrt(1-ori.w*ori.w)


def main():
    rospy.init_node('test', anonymous=False)
    signal.signal(signal.SIGINT, signal_handler)

    arm = ArmControl()
    arm.setSpeed(0.3)

    # Move to default pos
    arm.jointGoal([0, radians(-90), 0, 0, 0, 0])

    # Reset ft sensor
    rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor')
    try:
        zero = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)
        resp1 = zero()
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
    # for i in range (-180, 180, 20):
    #     arm.jointGoal([0, radians(-90), 0, 0, 0, radians(i)])
    #     time.sleep(5)
 
    # Move to default pos
    # arm.jointGoal([0, radians(-90), 0, 0, 0, 0])

    # Move wrist_3 in 1 degree steps
    # current_joints = arm.getJointValues()
    # for i in range(180):
    #     current_joints[5] = radians(i)
    #     arm.jointGoal(current_joints)
    #     print('')

    # Move to out of camera pos
    # rospy.wait_for_service('/cork_iris/control_arm')
    # try:
    #     alias = rospy.ServiceProxy('/cork_iris/control_arm', ControlArm)
    #     resp = alias(['out_of_camera'])
    #     print(resp.output_feedback)
    # except rospy.ServiceException as e:
    #     print("Service call failed: %s"%e)


    # Only move in XYZ - doesn't change orientation
    # arm.simpleMove([0, 0, -0.2])
    # arm.simpleMove([0, -0.2, 0])
    # arm.simpleMove([-0.2, 0, 0])
    # arm.simpleMove([0, 0, 0.4])
    # arm.simpleMove([0, 0.4, 0])
    # arm.simpleMove([0, 0, -0.4])
    # arm.simpleMove([0.4, 0, 0])
    # arm.simpleMove([0, -0.2, 0])
    # arm.simpleMove([-0.2, 0, 0])
    # arm.simpleMove([0, 0, 0.2])

    # Rotation
    # arm.simpleRotate([pi/4, 0, 0])
    # time.sleep(0.5)
    # arm.simpleRotate([pi/4, 0, 0])
    # time.sleep(0.5)
    # arm.simpleRotate([pi/4, 0, 0])
    # time.sleep(0.5)
    
    # arm.simpleRotate([-3*pi/4, 0, 0])
    # time.sleep(1)

    # arm.simpleRotate([-pi/4, 0, 0])
    # time.sleep(0.5)
    # arm.simpleRotate([-pi/4, 0, 0])
    # time.sleep(0.5)
    # arm.simpleRotate([-pi/4, 0, 0])
    # time.sleep(0.5)

    # arm.simpleRotate([3*pi/4, 0, 0])

    # Rotate then move XYZ
    # arm.simpleRotate([-pi/2, 0, 0])
    # arm.simpleMove([0, 0, -0.2])
    # arm.simpleMove([0, -0.2, 0])
    # arm.simpleMove([-0.2, 0, 0])
    # arm.simpleMove([0, 0, 0.4])
    # arm.simpleMove([0, 0.4, 0])
    # arm.simpleMove([0, 0, -0.4])
    # arm.simpleMove([0.4, 0, 0])
    # arm.simpleMove([0, -0.2, 0])
    # arm.simpleMove([-0.2, 0, 0])
    # arm.simpleMove([0, 0, 0.2])



if __name__ == "__main__":
    main()