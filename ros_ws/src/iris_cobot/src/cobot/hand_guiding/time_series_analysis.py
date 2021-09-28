#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import numpy as np

import cobot.helpers as helpers


WRENCH_WINDOW = 100
JOINT_WINDOW = 10
MAX_FORCE_MAGNITUDE = 2

wrench_fifo = np.empty([1, 6])
joint_fifo = np.empty([1, 6])

wrench_std_dev = None
joint_std_dev = None
wrench_move = None
joint_move = None
force_magnitude = None

events = []


def jointMonitor(data):
    positions = data.position
    if len(positions) != 6:
        return 

    global joint_fifo
    joint_fifo = np.append(joint_fifo, [positions], axis=0)

    std_dev = np.std(joint_fifo, axis=0)

    std_msg = Float64MultiArray()
    std_msg.layout.data_offset = 1
    std_msg.data = std_dev
    joint_std_dev.publish(std_msg)

    moving = 0
    if [x for x in std_dev if x > 0.0005] != []:
        moving = 1
        events.append(1)

    joint_move.publish(moving)

    if joint_fifo.shape[0] > JOINT_WINDOW:
        joint_fifo = np.delete(joint_fifo, 0, 0)
        

def wrenchMonitor(data):
    # Aquisition of the force values
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    t_x = data.wrench.torque.x
    t_y = data.wrench.torque.y
    t_z = data.wrench.torque.z

    # Filter wrench values
    global wrench_fifo
    wrench_fifo = np.append(wrench_fifo, [[f_x, f_y, f_z, t_x, t_y, t_z]], axis=0)

    std_dev = np.std(wrench_fifo, axis=0)

    std_msg = Float64MultiArray()
    std_msg.layout.data_offset = 1
    std_msg.data = std_dev
    wrench_std_dev.publish(std_msg)

    touching = 0
    if [x for x in std_dev if x > 0.8] != []:
        touching = 1
        events.append(1)

    wrench_move.publish(touching)

    if wrench_fifo.shape[0] > WRENCH_WINDOW:
        wrench_fifo = np.delete(wrench_fifo, 0, 0)


def forceMonitor(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    global force_magnitude
    force_magnitude = np.linalg.norm([f_x, f_y, f_z])


def zeroFTSensor(event):
    if len(events) == 0:
        if (force_magnitude > MAX_FORCE_MAGNITUDE):
            print('ZERO FT SENSOR')
            helpers.cobot_reset_ft_sensor()

    events[:] = []


def main():
    rospy.init_node('time_series_analysis', anonymous=True)

    global wrench_std_dev, joint_std_dev, wrench_move, joint_move
    wrench_std_dev = rospy.Publisher('wrench_std', Float64MultiArray, queue_size=1)
    joint_std_dev = rospy.Publisher('joint_std', Float64MultiArray, queue_size=1)

    wrench_move = rospy.Publisher('wrench_touch', Int32, queue_size=1)
    joint_move = rospy.Publisher('joint_move', Int32, queue_size=1)

    rospy.Subscriber('joint_states', JointState, jointMonitor)
    rospy.Subscriber("wrench", WrenchStamped, wrenchMonitor)
    rospy.Subscriber("wrench_correct", WrenchStamped, forceMonitor)

    rospy.Timer(rospy.Duration(1), zeroFTSensor)

    rospy.spin()

if __name__ == '__main__':
    main()