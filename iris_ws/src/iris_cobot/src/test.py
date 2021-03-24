#!/usr/bin/env python
import rospy
import signal, sys
from math import pi, sin, cos, acos, sqrt, radians
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import MarkerArray, Marker
from moveit_commander.move_group import MoveGroupCommander

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

    poses_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    markers = []

    # Move in various positions
    # angles = [-180, -135, -90, -45, 0, 45, 90, 135]
    # forbidden = [(45, -180),  (45, -135), (45, -90),
    #              (90, -180),  (90, -135), (90, 135),
    #              (135, -180), (135, 90),  (135, 135)]
    # idx = 0
    # for w_1 in angles:
    #     for w_2 in angles:
            
    #         if (w_1, w_2) not in forbidden:
    #             print('\nWrist 1 - %d' % w_1)
    #             print('Wrist 2 - %d' % w_2)
    #             print('Idx - %d' % idx)

    #             arm.move_joints([0, radians(-90), 0, radians(w_1), radians(w_2), 0])

    #             marker = Marker()
    #             marker.header.frame_id = "base_link"
    #             marker.header.stamp = rospy.Time()
    #             marker.id = idx
    #             idx += 1
    #             marker.type = marker.ARROW
    #             marker.action = marker.ADD
    #             marker.scale = Vector3(*[0.1, 0.01, 0.01])
    #             ee_pose = arm.get_pose()
    #             marker.pose = ee_pose
    #             print('Orientation - %s' % str(ee_pose.orientation))
    #             marker.color = ColorRGBA(*[1, 1, 0, 1])

    #             markers.append(marker)

    #             poses_pub.publish(markers)


    # Move to default pos
    arm.move_joints([0, radians(-90), 0, radians(135), radians(90), 0])

    # Reset ft sensor
    helpers.reset_ft_sensor()
    
    # for i in range (-180, 180, 20):
    #     arm.move_joints([0, radians(-90), 0, 0, 0, radians(i)])
    #     time.sleep(5)

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