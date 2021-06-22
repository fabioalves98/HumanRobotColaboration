#!/usr/bin/env python
import sys, signal

import rospy
import numpy as np
from math import degrees, radians, sin,  acos
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_multiply, quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3, WrenchStamped, Quaternion, TransformStamped
from moveit_commander.move_group import MoveGroupCommander

from iris_cobot.srv import WeightUpdate
from cobot.helpers import arrowMarker, quaternionToList, vectorFromQuaternion

GRIPPER_WEIGHT = 1.5
GRIPPER_COG = 0.042
OBJECT_WEIGHT = 0
OBJECT_COG = 0.165

gripper_weight = GRIPPER_WEIGHT
gripper_cog = GRIPPER_COG
object_weight = OBJECT_WEIGHT
object_cog = OBJECT_COG

def weightUpdateServ(data):
    global object_weight
    object_weight = data.weight
    return True


def signal_handler(sig, frame):
    print('')
    sys.exit(0)


def main():
    rospy.init_node('ft_theory', anonymous=False)
    signal.signal(signal.SIGINT, signal_handler)

    moveg = MoveGroupCommander('manipulator')

    br = tf2_ros.TransformBroadcaster()

    # m_x_pub = rospy.Publisher('mx_marker', Marker, queue_size=10)
    # m_y_pub = rospy.Publisher('my_marker', Marker, queue_size=10)
    # m_z_pub = rospy.Publisher('mz_marker', Marker, queue_size=10)
    m_g_pub = rospy.Publisher('mg_marker', Marker, queue_size=10)

    rospy.Service('weight_update', WeightUpdate, weightUpdateServ)

    wrench_pub = rospy.Publisher('wrench_theory', WrenchStamped, queue_size=1)

    x_rot = quaternion_from_euler(0, 0, radians(-90))
    y_rot = quaternion_from_euler(0, radians(90), 0)

    ft_rot = quaternion_from_euler(radians(-90), 0, radians(-90))

    rate = rospy.Rate(500)

    while not rospy.is_shutdown():
        ee_ori = moveg.get_current_pose().pose.orientation

        origin = [-0.7, 0.3, 0.5]

        # Arrow x
        x_ori = Quaternion(*quaternion_multiply(quaternionToList(ee_ori), x_rot))
        # x_pose = Pose(Point(*origin), x_ori)
        # m_x = arrowMarker(x_pose, ColorRGBA(*[1, 0, 0, 1]))

        # # Arrow y
        y_ori = Quaternion(*quaternion_multiply(quaternionToList(ee_ori), y_rot))
        # y_pose = Pose(Point(*origin), y_ori)
        # m_y = arrowMarker(y_pose, ColorRGBA(*[0, 1, 0, 1]))

        # # Arrow z
        # z_pose = Pose(Point(*origin),ee_ori)
        # m_z = arrowMarker(z_pose, ColorRGBA(*[0, 0, 1, 1]))


        # FT Orientation
        ft_ori = quaternion_multiply(quaternionToList(ee_ori), ft_rot)
        tcp_marker = TransformStamped()
        tcp_marker.header.stamp = rospy.Time.now()
        tcp_marker.header.frame_id = "base_link"
        tcp_marker.child_frame_id = "ft_sensor"
        tcp_marker.transform.translation = Vector3(*origin)
        tcp_marker.transform.rotation = Quaternion(*ft_ori)

        br.sendTransform(tcp_marker)

        # Gravity
        g_ori = Quaternion(*quaternion_from_euler(0, radians(90), 0))
        g_pose = Pose(Point(*origin), g_ori)
        m_g = arrowMarker(g_pose, ColorRGBA(*[1, 0, 0.5, 1]))

        # Publishers
        m_g_pub.publish(m_g)

        # Vectors
        v_x = vectorFromQuaternion(x_ori)
        v_y = vectorFromQuaternion(y_ori)
        v_z = vectorFromQuaternion(ee_ori)
        v_g = vectorFromQuaternion(g_ori)

        # FORCE
        # Obtain force
        total_weight = gripper_weight + object_weight
        f_x = np.inner(v_x, v_g) * total_weight * 10
        f_y = np.inner(v_y, v_g) * total_weight * 10
        f_z = np.inner(v_z, v_g) * total_weight * 10 

        # Correction Factors
        f_x = f_x * 0.846
        f_y = f_y * 1.115
        f_z += f_x * 0.154
        f_z = f_z * 1.230

        # TORQUE
        # Angle with gravity
        angle_w_g = acos(np.dot(v_z, v_g))
        
        # Normal of torque plane
        torque_plane_normal = np.cross(v_z, v_g)

        # Obtain torque
        total_force = gripper_weight * gripper_cog + object_weight * object_cog
        t_x = total_force * 10 * sin(angle_w_g) * np.dot(v_x, torque_plane_normal)
        t_y = total_force * 10 * sin(angle_w_g) * np.dot(v_y, torque_plane_normal)
        t_z = total_force * 10 * sin(angle_w_g) * np.dot(v_z, torque_plane_normal)
        
        # Wrench pub
        wrench = WrenchStamped()
        wrench.header.frame_id = "base_link"
        wrench.wrench.force = Vector3(*[f_x, f_y, f_z])
        wrench.wrench.torque = Vector3(*[t_x, t_y, t_z])

        wrench_pub.publish(wrench)

        rate.sleep()


if __name__ == "__main__":
    main()