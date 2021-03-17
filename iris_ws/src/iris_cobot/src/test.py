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
    m_p_x_pub = rospy.Publisher('mpx_marker', Marker, queue_size=10)
    m_y_pub = rospy.Publisher('my_marker', Marker, queue_size=10)
    m_p_y_pub = rospy.Publisher('mpy_marker', Marker, queue_size=10)
    m_z_pub = rospy.Publisher('mz_marker', Marker, queue_size=10)
    m_p_z_pub = rospy.Publisher('mpz_marker', Marker, queue_size=10)
    m_g_pub = rospy.Publisher('mg_marker', Marker, queue_size=10)
    m_p_g_pub = rospy.Publisher('mpg_marker', Marker, queue_size=10)

    wrench_pub = rospy.Publisher('wrench', WrenchStamped, queue_size=1)

    while not rospy.is_shutdown():
        origin = [0.5, 0.5, 0.5]

        # Arrow x
        m_x = Marker()
        m_x.header.frame_id = "base_link"
        m_x.type = m_x.ARROW
        m_x.action = m_x.ADD
        m_x.scale = Vector3(*[0.2, 0.02, 0.02])

        rot = quaternion_from_euler(0, 0, radians(-90))
        arm_ori = orientation_to_list(arm.get_pose().orientation)
        m_x_ori = quaternion_multiply(arm_ori, rot)
        m_x.pose.orientation = list_to_orientation(m_x_ori)

        m_x.pose.position  = Point(*origin)
        m_x.color = ColorRGBA(*[1, 0, 0, 1])

        # Arrow y
        m_y = Marker()
        m_y.header.frame_id = "base_link"
        m_y.type = m_y.ARROW
        m_y.action = m_y.ADD
        m_y.scale = Vector3(*[0.2, 0.02, 0.02])

        rot = quaternion_from_euler(0, radians(90), 0)
        arm_ori = orientation_to_list(arm.get_pose().orientation)
        m_y_ori = quaternion_multiply(arm_ori, rot)
        m_y.pose.orientation = list_to_orientation(m_y_ori)

        m_y.pose.position = Point(*origin)
        m_y.color = ColorRGBA(*[0, 1, 0, 1])

        # Arrow z
        m_z = Marker()
        m_z.header.frame_id = "base_link"
        m_z.type = m_z.ARROW
        m_z.action = m_z.ADD
        m_z.scale = Vector3(*[0.2, 0.02, 0.02])

        m_z.pose.orientation = arm.get_pose().orientation
        m_z.pose.position = Point(*origin)
        m_z.color = ColorRGBA(*[0, 0, 1, 1])

        # Gravity
        m_g = Marker()
        m_g.header.frame_id = "base_link"
        m_g.type = m_g.ARROW
        m_g.action = m_g.ADD
        m_g.scale = Vector3(*[0.2, 0.02, 0.02])
        m_g.pose.orientation = list_to_orientation(quaternion_from_euler(0, radians(90), 0))
        m_g.pose.position = Point(*origin)
        m_g.color = ColorRGBA(*[1, 0, 0.5, 1])

        # Auxiliary position for vector creation
        aux_p = PoseStamped()
        aux_p.header.frame_id = "base_link"
        aux_p.pose.position = Point(*[0.3, 0, 0])
        aux_p.pose.orientation = Quaternion(*[0, 0, 0, 1])

        # Point x
        t_x = TransformStamped()
        t_x.header.frame_id = "base_link"
        t_x.transform.translation = Vector3(*origin)
        t_x.transform.rotation = m_x.pose.orientation

        p_x_new = do_transform_pose(aux_p, t_x)

        m_p_x = Marker()
        m_p_x.header.frame_id = "base_link"
        m_p_x.type = m_p_x.SPHERE
        m_p_x.action = m_p_x.ADD
        m_p_x.scale = Vector3(*[0.03, 0.03, 0.03])
        m_p_x.pose = p_x_new.pose
        m_p_x.color = ColorRGBA(*[1, 0, 0, 1])

        # Point y 
        t_y = TransformStamped()
        t_y.header.frame_id = "base_link"
        t_y.transform.translation = Vector3(*origin)
        t_y.transform.rotation = m_y.pose.orientation

        p_y_new = do_transform_pose(aux_p, t_y)

        m_p_y = Marker()
        m_p_y.header.frame_id = "base_link"
        m_p_y.type = m_p_y.SPHERE
        m_p_y.action = m_p_y.ADD
        m_p_y.scale = Vector3(*[0.03, 0.03, 0.03])
        m_p_y.pose = p_y_new.pose
        m_p_y.color = ColorRGBA(*[0, 1, 0, 1])

        # Point z
        t_z = TransformStamped()
        t_z.header.frame_id = "base_link"
        t_z.transform.translation = Vector3(*origin)
        t_z.transform.rotation = m_z.pose.orientation

        p_z_new = do_transform_pose(aux_p, t_z)

        m_p_z = Marker()
        m_p_z.header.frame_id = "base_link"
        m_p_z.type = m_p_z.SPHERE
        m_p_z.action = m_p_z.ADD
        m_p_z.scale = Vector3(*[0.03, 0.03, 0.03])
        m_p_z.pose = p_z_new.pose
        m_p_z.color = ColorRGBA(*[0, 0, 1, 1])

        # Point g
        t_g = TransformStamped()
        t_g.header.frame_id = "base_link"
        t_g.transform.translation = Vector3(*origin)
        t_g.transform.rotation = m_g.pose.orientation

        p_g_new = do_transform_pose(aux_p, t_g)

        m_p_g = Marker()
        m_p_g.header.frame_id = "base_link"
        m_p_g.type = m_p_g.SPHERE
        m_p_g.action = m_p_g.ADD
        m_p_g.scale = Vector3(*[0.03, 0.03, 0.03])
        m_p_g.pose = p_g_new.pose
        m_p_g.color = ColorRGBA(*[1, 0, 0.5, 1])

        # Publishers
        m_x_pub.publish(m_x)
        m_p_x_pub.publish(m_p_x)
        m_y_pub.publish(m_y)
        m_p_y_pub.publish(m_p_y)
        m_z_pub.publish(m_z)
        m_p_z_pub.publish(m_p_z)
        m_g_pub.publish(m_g)
        m_p_g_pub.publish(m_p_g)

        # Vectors
        v_x = (np.array(point_to_list(p_x_new.pose.position)) - np.array(origin))
        v_y = (np.array(point_to_list(p_y_new.pose.position)) - np.array(origin))
        v_z = (np.array(point_to_list(p_z_new.pose.position)) - np.array(origin))
        v_g = (np.array(point_to_list(p_g_new.pose.position)) - np.array(origin))

        # Normalize
        v_x = v_x / np.linalg.norm(v_x)
        v_y = v_y / np.linalg.norm(v_y)
        v_z = v_z / np.linalg.norm(v_z)
        v_g = v_g / np.linalg.norm(v_g)
        
        # Obtain force
        f_x = np.inner(v_x, v_g) * 15
        f_y = np.inner(v_y, v_g) * 15
        f_z = np.inner(v_z, v_g) * 15

        # Wrench pub
        wrench = WrenchStamped()
        wrench.header.frame_id = "base_link"
        wrench.wrench.force = Vector3(*[f_x, f_y, f_z])

        wrench_pub.publish(wrench)

        rospy.sleep(0.002)


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