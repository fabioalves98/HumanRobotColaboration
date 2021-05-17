#!/usr/bin/env python
import rospy
import numpy as np
from math import radians
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import WrenchStamped, Vector3, Pose, Quaternion, Point, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import quaternion_multiply, quaternion_from_euler
from moveit_commander.move_group import MoveGroupCommander

from helpers import pointToList, quaternionToList

weight_marker = None
weight_pub = None
moveg = None


def weightMarker(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    origin = [-0.7, 0.3, 0.5]
    ee_ori = moveg.get_current_pose().pose.orientation

    rot = quaternion_from_euler(radians(-90), 0, radians(-90))
    w_ori = quaternion_multiply(quaternionToList(ee_ori), rot)

    # Point in space with coordinates representing the weight
    w_pose = PoseStamped()
    w_pose.header.frame_id = "base_link"
    w_pose.pose.position = Point(*[f_x/50, f_y/50, f_z/50])
    w_pose.pose.orientation = Quaternion(*[0, 0, 0, 1])

    # Transform such point to visualization axes
    t_w = TransformStamped()
    t_w.header.frame_id = "base_link"
    t_w.transform.translation = Vector3(*origin)
    t_w.transform.rotation = Quaternion(*w_ori)

    p_w_new = do_transform_pose(w_pose, t_w)

    # Create and normalize vector
    v_w = np.array(pointToList(p_w_new.pose.position) - np.array(origin))
    # v_w = v_w / np.linalg.norm(v_w)

    # Create point marker
    m_w = Marker()
    m_w.header.frame_id = "base_link"
    m_w.type = m_w.SPHERE
    m_w.action = m_w.ADD
    m_w.scale = Vector3(*[0.03, 0.03, 0.03])
    m_w.pose = p_w_new.pose
    m_w.color = ColorRGBA(*[1, 1, 1, 1])

    # Publish vector and marker
    weight_pub.publish(Vector3(*v_w))
    weight_marker.publish(m_w)


def main():
    rospy.init_node('force_to_linvel', anonymous=True)

    global weight_marker, weight_pub, moveg
    weight_marker = rospy.Publisher('w_marker', Marker, queue_size=1)
    weight_pub = rospy.Publisher('linear_velocity', Vector3, queue_size=1)

    moveg = MoveGroupCommander('manipulator')

    rospy.Subscriber('wrench_correct', WrenchStamped, weightMarker, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()