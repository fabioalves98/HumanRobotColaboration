#!/usr/bin/env python
import rospy, rospkg, pickle
import numpy as np
from ur_msgs.srv import SetSpeedSliderFraction
from std_srvs.srv import Trigger
from geometry_msgs.msg import Quaternion

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


def point_to_list(point):
    p_list = []
    p_list.append(point.x)
    p_list.append(point.y)
    p_list.append(point.z)
    return p_list


def orientation_to_list(orientation):
    ori = []
    ori.append(orientation.x)
    ori.append(orientation.y)
    ori.append(orientation.z)
    ori.append(orientation.w)
    return ori


def list_to_orientation(ori_list):
    orientation = Quaternion()
    orientation.x = ori_list[0]
    orientation.y = ori_list[1]
    orientation.z = ori_list[2]
    orientation.w = ori_list[3]
    return orientation

