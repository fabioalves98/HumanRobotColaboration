#!/usr/bin/env python
import sys, signal
import rospy
import numpy as np
from math import radians
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_multiply, quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3, WrenchStamped, Quaternion
from moveit_commander.move_group import MoveGroupCommander

from helpers import arrowMarker, quaternionToList, vectorFromQuaternion


def signal_handler(sig, frame):
    print('')
    sys.exit(0)


def main():
    rospy.init_node('ft_sensor', anonymous=False)
    signal.signal(signal.SIGINT, signal_handler)

    moveg = MoveGroupCommander('manipulator')

    m_x_pub = rospy.Publisher('mx_marker', Marker, queue_size=10)
    m_y_pub = rospy.Publisher('my_marker', Marker, queue_size=10)
    m_z_pub = rospy.Publisher('mz_marker', Marker, queue_size=10)
    m_g_pub = rospy.Publisher('mg_marker', Marker, queue_size=10)

    wrench_pub = rospy.Publisher('wrench_theory', WrenchStamped, queue_size=1)

    weight = 1.5

    while not rospy.is_shutdown():
        ee_ori = moveg.get_current_pose().pose.orientation

        origin = [-0.7, 0.3, 0.5]

        # Arrow x
        x_rot = quaternion_from_euler(0, 0, radians(-90))
        x_ori = Quaternion(*quaternion_multiply(quaternionToList(ee_ori), x_rot))
        x_pose = Pose(Point(*origin), x_ori)
        m_x = arrowMarker(x_pose, ColorRGBA(*[1, 0, 0, 1]))

        # Arrow y
        y_rot = quaternion_from_euler(0, radians(90), 0)
        y_ori = Quaternion(*quaternion_multiply(quaternionToList(ee_ori), y_rot))
        y_pose = Pose(Point(*origin), y_ori)
        m_y = arrowMarker(y_pose, ColorRGBA(*[0, 1, 0, 1]))

        # Arrow z
        z_pose = Pose(Point(*origin),ee_ori)
        m_z = arrowMarker(z_pose, ColorRGBA(*[0, 0, 1, 1]))

        # Gravity
        g_ori = Quaternion(*quaternion_from_euler(0, radians(90), 0))
        g_pose = Pose(Point(*origin), g_ori)
        m_g = arrowMarker(g_pose, ColorRGBA(*[1, 0, 0.5, 1]))

        # Publishers
        m_x_pub.publish(m_x)
        m_y_pub.publish(m_y)
        m_z_pub.publish(m_z)
        m_g_pub.publish(m_g)

        # Vectors
        v_x = vectorFromQuaternion(x_ori)
        v_y = vectorFromQuaternion(y_ori)
        v_z = vectorFromQuaternion(ee_ori)
        v_g = vectorFromQuaternion(g_ori)

        # Obtain force
        f_x = np.inner(v_x, v_g) * weight * 10
        f_y = np.inner(v_y, v_g) * weight * 10
        f_z = np.inner(v_z, v_g) * weight * 10 

        # Correction Factors
        f_x = f_x * 0.846
        f_y = f_y * 1.115
        f_z += f_x * 0.154
        f_z = f_z * 1.230
        
        # Wrench pub
        wrench = WrenchStamped()
        wrench.header.frame_id = "base_link"
        wrench.wrench.force = Vector3(*[f_x, f_y, f_z])

        wrench_pub.publish(wrench)

        rospy.sleep(0.002)


if __name__ == "__main__":
    main()