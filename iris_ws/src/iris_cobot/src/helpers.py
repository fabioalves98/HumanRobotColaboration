#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger

def reset_ft_sensor():
    try:
        rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor', 2)
        zero = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)
        resp = zero()
        print(resp)
    except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
        print("Service call failed: %s"%e)

def plotXYZ(plt, x, array, line=''):
    plt.plot(x, [t[0] for t in array], 'r' + line)
    plt.plot(x, [t[1] for t in array], 'g' + line)
    plt.plot(x, [t[2] for t in array], 'b' + line)