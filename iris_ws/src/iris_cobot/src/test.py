#!/usr/bin/env python
import rospy
import time, signal, sys
from math import pi, sin, cos, acos, sqrt, radians
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_multiply, quaternion_from_euler
from moveit_commander.conversions import list_to_pose, pose_to_list

from sami.arm import Arm # pylint: disable=import-error, no-name-in-module
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

    m_x_pub = rospy.Publisher('mx_marker', Marker, queue_size=10)
    m_y_pub = rospy.Publisher('my_marker', Marker, queue_size=10)
    m_z_pub = rospy.Publisher('mz_marker', Marker, queue_size=10)
    m_g_pub = rospy.Publisher('mg_marker', Marker, queue_size=10)

    while not rospy.is_shutdown():
        # Marker x
        m_x = Marker()
        m_x.header.frame_id = "base_link"
        m_x.type = m_x.ARROW
        m_x.action = m_x.ADD
        m_x.scale.x = 0.2
        m_x.scale.y = 0.02
        m_x.scale.z = 0.02
        m_x.color.a = 1.0

        rot = quaternion_from_euler(0, 0, radians(-90))
        arm_ori = orientation_to_list(arm.get_pose().orientation)
        m_x_ori = quaternion_multiply(arm_ori, rot)
        m_x.pose.orientation = list_to_orientation(m_x_ori)

        m_x.pose.position.x = 0.5
        m_x.pose.position.y = 0.5
        m_x.pose.position.z = 0.5
        m_x.color.r = 1
        m_x.color.g = 0
        m_x.color.b = 0

        # Marker y
        m_y = Marker()
        m_y.header.frame_id = "base_link"
        m_y.type = m_y.ARROW
        m_y.action = m_y.ADD
        m_y.scale.x = 0.2
        m_y.scale.y = 0.02
        m_y.scale.z = 0.02
        m_y.color.a = 1.0

        rot = quaternion_from_euler(0, radians(90), 0)
        arm_ori = orientation_to_list(arm.get_pose().orientation)
        m_y_ori = quaternion_multiply(arm_ori, rot)
        m_y.pose.orientation = list_to_orientation(m_y_ori)

        m_y.pose.position.x = 0.5
        m_y.pose.position.y = 0.5
        m_y.pose.position.z = 0.5
        m_y.color.r = 0
        m_y.color.g = 1
        m_y.color.b = 0

        # Marker z
        m_z = Marker()
        m_z.header.frame_id = "base_link"
        m_z.type = m_z.ARROW
        m_z.action = m_z.ADD
        m_z.scale.x = 0.2
        m_z.scale.y = 0.02
        m_z.scale.z = 0.02
        m_z.color.a = 1.0
        m_z.pose.orientation = arm.get_pose().orientation
        m_z.pose.position.x = 0.5
        m_z.pose.position.y = 0.5
        m_z.pose.position.z = 0.5
        m_z.color.r = 0
        m_z.color.g = 0
        m_z.color.b = 1

        # Gravity
        m_g = Marker()
        m_g.header.frame_id = "base_link"
        m_g.type = m_g.ARROW
        m_g.action = m_g.ADD
        m_g.scale.x = 0.2
        m_g.scale.y = 0.02
        m_g.scale.z = 0.02
        m_g.color.a = 1.0
        m_g.pose.orientation = list_to_orientation(quaternion_from_euler(0, radians(90), 0))
        m_g.pose.position.x = 0.5
        m_g.pose.position.y = 0.5
        m_g.pose.position.z = 0.5
        m_g.color.r = 1
        m_g.color.g = 0
        m_g.color.b = 0.5

        m_x_pub.publish(m_x)
        m_y_pub.publish(m_y)
        m_z_pub.publish(m_z)
        m_g_pub.publish(m_g)


        # Vectors
        v_g = np.array([])
        
        rospy.sleep(0.01)


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