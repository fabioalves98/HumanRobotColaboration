#!/usr/bin/env python
import rospy
from math import radians
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import WrenchStamped, Vector3, Pose, Quaternion, Point, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import quaternion_multiply, quaternion_from_euler
from moveit_commander.move_group import MoveGroupCommander

from helpers import orientation_to_list, list_to_orientation

weight_marker = None
moveg = None


def weightMarker(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    origin = [0.6, 0.6, 1.4]
    ee_ori = moveg.get_current_pose().pose.orientation

    rot = quaternion_from_euler(radians(-90), 0, radians(-90))
    w_ori = quaternion_multiply(orientation_to_list(ee_ori), rot)

    w_pose = PoseStamped()
    w_pose.header.frame_id = "base_link"
    w_pose.pose.position = Point(*[f_x/50, f_y/50, f_z/50])
    w_pose.pose.orientation = Quaternion(*[0, 0, 0, 1])

    t_w = TransformStamped()
    t_w.header.frame_id = "base_link"
    t_w.transform.translation = Vector3(*origin)
    t_w.transform.rotation = list_to_orientation(w_ori)

    p_w_new = do_transform_pose(w_pose, t_w)


    w_m = Marker()
    w_m.header.frame_id = "base_link"
    w_m.type = w_m.SPHERE
    w_m.action = w_m.ADD
    w_m.scale = Vector3(*[0.03, 0.03, 0.03])
    w_m.pose = p_w_new.pose
    w_m.color = ColorRGBA(*[1, 1, 1, 1])

    weight_marker.publish(w_m)


def main():
    rospy.init_node('weight', anonymous=True)

    global weight_marker, moveg
    weight_marker = rospy.Publisher('w_marker', Marker, queue_size=1)

    moveg = MoveGroupCommander('manipulator')

    wrench_sub = rospy.Subscriber('wrench_correct', WrenchStamped, weightMarker, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()