#!/usr/bin/env python
from numpy.core.numeric import full
import rospy, rospkg
import signal, sys, os, time, timeit
from math import degrees, pi, sin, cos, acos, sqrt, radians

from sami.arm import Arm
import helpers

arm = None


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

    print(qx, qy, qz, qw)


def quaternionToAxisAngle():
    # Quaternion to Axis Angle
    pose = arm.get_joints()
    print(pose.orientation)
    
    ori = pose.orientation

    angle = 2 * acos(ori.w)
    x = ori.x / sqrt(1-ori.w*ori.w)
    y = ori.y / sqrt(1-ori.w*ori.w)
    z = ori.z / sqrt(1-ori.w*ori.w)

    print(angle, x, y, z)


def main():
    rospy.init_node('test', anonymous=False)
    signal.signal(signal.SIGINT, signal_handler)

    global arm

    arm = Arm('ur10e_moveit', group='manipulator', joint_positions_filename="positions.yaml")
    arm.velocity = 1

    # Rotate wrist_3
    # for i in range(-180, 180, 30):
    #     print('Wrist_3 - %d' % i)
    #     joints = arm.get_joints()
    #     joints[5] = radians(i)
    #     arm.move_joints(joints)
    #     raw_input("Move next? ")

    # Move to default pos
    arm.move_joints([0, radians(-90), 0, radians(0), radians(-90), 0])

    # Reset ft sensor
    # helpers.reset_ft_sensor()

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
    # time.sleep(1)

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