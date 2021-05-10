#!/usr/bin/env python
import rospy, tf2_ros
import numpy as np
from math import radians
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import WrenchStamped, Vector3, Pose, Quaternion, Point, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import quaternion_multiply, quaternion_from_euler, inverse_matrix
from moveit_commander.move_group import MoveGroupCommander

from helpers import pointToList, quaternionToList, arrowMarker

torque_marker = None
torque_pub = None
moveg = None


def torqueRotation(data):
    t_x = data.wrench.torque.x
    t_y = data.wrench.torque.y
    t_z = data.wrench.torque.z

    if t_x > radians(45):
        t_x = radians(45)
    if t_x < -radians(45):
        t_x = -radians(45)
    if t_y > radians(45):
        t_y = radians(45)
    if t_y < -radians(45):
        t_y = -radians(45)
    if t_z > radians(45):
        t_z = radians(45)
    if t_z < -radians(45):
        t_z = -radians(45)
    
    br = tf2_ros.TransformBroadcaster()

    origin = [-0.7, 0.3, 0.5]
    ee_ori = moveg.get_current_pose().pose.orientation

    rot = quaternion_from_euler(radians(-90), 0, radians(-90))
    w_ori = quaternion_multiply(quaternionToList(ee_ori), rot)

    # Point in space with coordinates representing the weight
    w_pose = PoseStamped()
    w_pose.header.frame_id = "base_link"
    w_pose.pose.position = Point(*[0, 0, 0])
    w_pose.pose.orientation = Quaternion(*quaternion_from_euler(t_x, t_y, t_z))

    # Transform such point to visualization axes
    t_w = TransformStamped()
    t_w.header.frame_id = "base_link"
    t_w.transform.translation = Vector3(*origin)
    t_w.transform.rotation = Quaternion(*w_ori)

    p_w_new = do_transform_pose(w_pose, t_w)

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "torque"
    t.transform.translation = Vector3(*origin)
    t.transform.rotation = p_w_new.pose.orientation
    br.sendTransform(t)

    # Create and normalize vector
    # rot_q = quaternion_multiply(quaternionToList(p_w_new.pose.orientation), 
    #                             quaternion_inverse(w_ori))
    # print(euler_from_quaternion(rot_q))


def main():
    rospy.init_node('torque', anonymous=True)

    global torque_marker, torque_pub, moveg
    moveg = MoveGroupCommander('manipulator')

    rospy.Subscriber('wrench', WrenchStamped, torqueRotation, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()