#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

rospy.init_node('velocity_node')
pub=rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)


msg=Float64MultiArray()
msg.data=[-0.1,0,0,0,0,0]
msg.layout.data_offset=1

rate=rospy.Rate(100)

raw_input("Posso?")
for i in range(200):
    pub.publish(msg)
    rate.sleep()

msg=Float64MultiArray()
msg.data=[0,0,0,0,0,0]
msg.layout.data_offset=1

pub.publish(msg)
