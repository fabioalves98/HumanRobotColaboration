#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3
from std_srvs.srv import Trigger

from iris_cobot.msg import PFVector
from iris_cobot.srv import WrenchControl

attraction_vel = None
repulsion_vel = None

atraction_clk = None
repulsion_clk = None

wrench_vel_pub = None # type: rospy.Publisher

status = "stop"

def attraction_cb(attraction_msg):
    attraction_vel.linear_velocity = attraction_msg.linear_velocity
    attraction_vel.angular_velocity = attraction_msg.angular_velocity
    global attraction_clk
    attraction_clk = rospy.Time.now()


def repulsion_cb(repulsion_msg):
    repulsion_vel.linear_velocity = repulsion_msg.linear_velocity
    global repulsion_clk
    repulsion_clk = rospy.Time.now()


def topic_clean(event):
    current_time = rospy.Time.now()
    if (current_time - attraction_clk).to_sec() > 0.1:
        attraction_vel.linear_velocity = Vector3(0, 0, 0)
        attraction_vel.angular_velocity = Vector3(0, 0, 0)
    if (current_time - repulsion_clk).to_sec() > 0.1:
        repulsion_vel.linear_velocity = Vector3(0, 0, 0)


def pf_control(data):
    global status
    if data.control in ["play", "pause", "stop"]:
        status = data.control
        return True
    else:
        return False


def main():
    rospy.init_node('pf_controller', anonymous=False)

    global attraction_vel, repulsion_vel, attraction_clk, repulsion_clk
    attraction_vel = PFVector()
    repulsion_vel = PFVector()
    attraction_clk = rospy.Time.now()
    repulsion_clk = rospy.Time.now()

    # Potential fields control service 
    s = rospy.Service('pf_control', WrenchControl, pf_control)

    # Attraction and repulsion vector subscribers
    rospy.Subscriber('attraction', PFVector, attraction_cb)
    rospy.Subscriber('repulsion', PFVector, repulsion_cb)

    # Make sure controller gets updated values, otherwise stops movement
    rospy.Timer(rospy.Duration(0.1), topic_clean)

    # Wrench velocity publisher
    global wrench_vel_pub
    wrench_vel_pub = rospy.Publisher('wrench_velocity', WrenchStamped, queue_size=1)

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
        pf_vel = np.add((1 - repulsion_mgn) * attraction, repulsion_mgn * repulsion)

        # Normalize linear velocity
        if np.any(pf_vel[:3]):
            pf_lin_vel = pf_vel[:3] / np.linalg.norm(pf_vel[:3]) * 0.5
        else:
            pf_lin_vel = pf_vel[:3]
        
        # Normalize angular velocity
        if np.any(pf_vel[3:]):
            pf_ang_vel = pf_vel[3:] / np.linalg.norm(pf_vel[3:])
        else:
            pf_ang_vel = pf_vel[3:]

        if status == "play":
            wrench_vel_msg = WrenchStamped()
            wrench_vel_msg.wrench.force = Vector3(*pf_lin_vel)
            wrench_vel_msg.wrench.torque = Vector3(*pf_ang_vel)
            wrench_vel_pub.publish(wrench_vel_msg)
        elif status == "pause":
            wrench_vel_pub.publish(WrenchStamped())
        elif status == "stop":
            pass
            
        rate.sleep()


if __name__ == "__main__":
    main()