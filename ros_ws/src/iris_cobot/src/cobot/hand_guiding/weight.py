#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32

import cobot.helpers as helpers

weight_pub = None
weight_correct_pub = None


def wrench_weight_calc(data):
    force = data.wrench.force
    weight = math.sqrt(math.pow(force.x, 2) + math.pow(force.y, 2) + math.pow(force.z, 2)) / helpers.G
    weight_pub.publish(Float32(weight))


def wrench_weight_correct_calc(data):
    force = data.wrench.force
    weight = math.sqrt(math.pow(force.x, 2) + math.pow(force.y, 2) + math.pow(force.z, 2)) / helpers.G
    weight_correct_pub.publish(Float32(weight))


def main():
    rospy.init_node('weight', anonymous=True)

    global weight_pub, weight_correct_pub
    weight_pub = rospy.Publisher('weight', Float32, queue_size=1)
    weight_correct_pub = rospy.Publisher('weight_correct', Float32, queue_size=1)

    rospy.Subscriber("wrench_filtered", WrenchStamped, wrench_weight_calc)
    rospy.Subscriber("wrench_correct", WrenchStamped, wrench_weight_correct_calc)

    rospy.spin()


if __name__ == '__main__':
    main()