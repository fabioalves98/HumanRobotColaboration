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
    ft_sensor_ori = quaternion_multiply(quaternionToList(ee_ori), rot)

    # Point in space with coordinates representing the weight
    torque_pose = PoseStamped()
    torque_pose.header.frame_id = "world"
    torque_pose.pose.position = Point(*[0, 0, 0])
    torque_pose.pose.orientation = Quaternion(*quaternion_from_euler(t_x, t_y, t_z))

    # Transform such point to visualization axes
    ft_sensor_transform = TransformStamped()
    ft_sensor_transform.header.frame_id = "world"
    ft_sensor_transform.transform.translation = Vector3(*origin)
    ft_sensor_transform.transform.rotation = Quaternion(*ft_sensor_ori)

    torque_pose_new = do_transform_pose(torque_pose, ft_sensor_transform)

    torque_transform = TransformStamped()
    torque_transform.header.stamp = rospy.Time.now()
    torque_transform.header.frame_id = "world"
    torque_transform.child_frame_id = "torque"
    torque_transform.transform.translation = Vector3(*origin)
    torque_transform.transform.rotation = torque_pose_new.pose.orientation

    # print()

    br.sendTransform(torque_transform)

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