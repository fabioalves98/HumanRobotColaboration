#!/usr/bin/env python
import rospy
import time, signal, sys
from math import pi, sin, cos, acos, sqrt, radians
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Vector3, WrenchStamped
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_multiply, quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from moveit_commander.conversions import list_to_pose, pose_to_list

from sami.arm import Arm
from helpers import *

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
    arm.velocity = 0.2

    # Test
    arm.move_pose([0.5, 0.5, 0.8, 0, 0, 0])

    # Move in various positions
    # angles = [-180, -135, -90, -45, 0, 45, 90, 135, 180]
    # forbidden = [(45, -180),  (45, -135), (45, -90),  (45, 18c0),
    #              (90, -180),  (90, -135), (90, 135),  (90, 180),
    #              (135, -180), (135, 90),  (135, 135), (135, 180)]

    # for w_1 in angles:
    #     for w_2 in angles:
    #         print('\nWrist 1 - %d' % w_1)
    #         print('Wrist 2 - %d' % w_2)
    #         if (w_1, w_2) not in forbidden:
    #             arm.move_joints([0, radians(-90), 0, radians(w_1), radians(w_2), 0])


    # Reset ft sensor
    # reset_ft_sensor()
    
    # for i in range (-180, 180, 20):
    #     arm.move_joints([0, radians(-90), 0, 0, 0, radians(i)])
    #     time.sleep(5)
 
    # Move to default pos
    # arm.move_joints([0, radians(-90), 0, 0, 0, 0])

    # Move wrist_3 in 1 degree steps
    # current_joints = arm.getJointValues()
    # for i in range(180):
    #     current_joints[5] = radians(i)
    #     arm.move_joints(current_joints)
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