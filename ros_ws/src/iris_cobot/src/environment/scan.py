#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
from math import radians

import cobot.helpers as helpers


def takeSampleService():
    rospy.wait_for_service('iris_cobot/environment/take_sample')
    try:
        take_sample_serv = rospy.ServiceProxy('iris_cobot/environment/take_sample', Empty)
        resp = take_sample_serv()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main():
    rospy.init_node('scan', anonymous=False)

    helpers.samiJointService([0, radians(-90), 0, 0, 0, radians(90)])

    print("Started taking samples")

    for s_p in [-135, -45, 45, 135]:
        for w_2 in range(-180, 180, 45):
            helpers.samiJointService([radians(s_p), radians(-90), radians(-90), 
                                                 0, radians(w_2), radians(90)])
            takeSampleService()

    print("Finished taking samples")
    
    rospy.spin()

if __name__ == "__main__":
    main()