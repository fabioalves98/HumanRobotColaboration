#!/usr/bin/env python
import rospy, rospkg, pickle
import numpy as np
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from ur_msgs.srv import SetSpeedSliderFraction
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped
from visualization_msgs.msg import Marker

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')


def set_speed_slider(self, speed):
    try:
        rospy.wait_for_service('/ur_hardware_interface/set_speed_slider', timeout=2)
        set_speed = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
        resp = set_speed(speed)
        print(resp)
    except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
        print("Service call failed: %s"%e)


def reset_ft_sensor():
    try:
        rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor', timeout=2)
        zero = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)
        resp = zero()
        print(resp)
    except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
        print("Service call failed: %s"%e)


def openList(filename):    
    with open(BASE_DIR + filename) as f:
        return np.array(pickle.load(f))


def plotXYZ(plt, x, array, line='', alpha=1, title=''):
    array = np.array(array)

    if array.shape == (3, 360):
        array = array.transpose()

    plt.plot(x, [t[0] for t in array], 'r' + line, alpha=alpha)
    plt.plot(x, [t[1] for t in array], 'g' + line, alpha=alpha)
    plt.plot(x, [t[2] for t in array], 'b' + line, alpha=alpha)
    
    if title:
        plt.set_title(title, loc='center')


def arrowMarker(pose, color):
    ''' Returns an ROS arrow marker
    Parameters:
        pose -          geometry_msgs Pose
        color -         std_msgs ColorRGBA
    '''
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale = Vector3(*[0.2, 0.02, 0.02])
    marker.pose = pose
    marker.color = color

    return marker

def vectorFromQuaternion(quaternion):
    ''' Returns a normalized vector form a quaternion
    Parameters:
        quaterion -     geometry_msgs Quaternion
        color -         std_msgs ColorRGBA
    '''
    initial_pose = PoseStamped(pose=Pose(Point(*[1, 0, 0]), Quaternion(*[0, 0, 0, 1])))

    transform = TransformStamped(transform=Transform(Vector3(*[0, 0, 0]), quaternion))

    new_pose = do_transform_pose(initial_pose, transform)

    return pointToList(new_pose.pose.position)


def pointToList(point):
    p_list = []
    p_list.append(point.x)
    p_list.append(point.y)
    p_list.append(point.z)
    return p_list


def quaternionToList(orientation):
    ori = []
    ori.append(orientation.x)
    ori.append(orientation.y)
    ori.append(orientation.z)
    ori.append(orientation.w)
    return ori


