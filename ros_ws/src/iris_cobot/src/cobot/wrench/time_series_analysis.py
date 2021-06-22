#!/usr/bin/env python
import rospy, rospkg, signal, sys
from std_msgs.msg import Float64MultiArray, Int32
from geometry_msgs.msg import WrenchStamped
import numpy as np


MEAN_WINDOW = 100
wrench_fifo = np.empty([1, 6])

wrench_std_dev = None
wrench_move = None

def wrenchFilter(data):
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
    if [x for x in std_dev if x > 0.2] != []:
        touching = 1

    wrench_move.publish(touching)

    if wrench_fifo.shape[0] > MEAN_WINDOW:
        wrench_fifo = np.delete(wrench_fifo, 0, 0)
         

def main():
    rospy.init_node('time_series_analysis', anonymous=True)

    global wrench_std_dev, wrench_move
    wrench_std_dev = rospy.Publisher('wrench_std', Float64MultiArray, queue_size=1)
    wrench_move = rospy.Publisher('wrench_touch', Int32, queue_size=1)

    rospy.Subscriber("wrench_filtered", WrenchStamped, wrenchFilter)

    rospy.spin()


if __name__ == '__main__':
    main()