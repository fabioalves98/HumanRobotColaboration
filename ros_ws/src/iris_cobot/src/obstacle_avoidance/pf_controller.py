#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3

from iris_cobot.msg import PFVector

attraction_vel = None
repulsion_vel = None

def attraction_cb(attraction_msg):
    attraction_vel.linear_velocity = attraction_msg.linear_velocity
    attraction_vel.angular_velocity = attraction_msg.angular_velocity


def repulsion_cb(repulsion_msg):
    repulsion_vel.linear_velocity = repulsion_msg.linear_velocity


def main():
    rospy.init_node('pf_controller', anonymous=False)

    global attraction_vel, repulsion_vel
    attraction_vel = PFVector()
    repulsion_vel = PFVector()

    # Attraction and repulsion vector subscribers
    rospy.Subscriber('attraction', PFVector, attraction_cb)
    rospy.Subscriber('repulsion', PFVector, repulsion_cb)

    # Linear and angular velocity publisher
    linear_vel_pub = rospy.Publisher('linear_velocity', Vector3, queue_size=1)
    angular_vel_pub = rospy.Publisher('angular_velocity', Vector3, queue_size=1)

    rospy.loginfo("pf_controller node listening to attraction/repulsion and publishing to \
linear/angular velocity")

    # Vel = (1 - ||Rv||) Av + Rv

    rate = rospy.Rate(500)

    while not rospy.is_shutdown():    
        repulsion = np.array([repulsion_vel.linear_velocity.x, repulsion_vel.linear_velocity.y,
                              repulsion_vel.linear_velocity.z, 0, 0, 0])
        repulsion_mgn = np.linalg.norm(repulsion)

        attraction = np.array([attraction_vel.linear_velocity.x, attraction_vel.linear_velocity.y,
                               attraction_vel.linear_velocity.z, attraction_vel.angular_velocity.x, 
                               attraction_vel.angular_velocity.y, attraction_vel.angular_velocity.z])
        
        # Potential feilds equation
        pf_vel = np.add((1 - repulsion_mgn) * attraction, repulsion)

        linear_vel_pub.publish(Vector3(*pf_vel[:3]))
        angular_vel_pub.publish(Vector3(*pf_vel[3:]))

        rate.sleep()


if __name__ == "__main__":
    main()