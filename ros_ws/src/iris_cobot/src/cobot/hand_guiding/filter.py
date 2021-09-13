#!/usr/bin/env python
import rospy, rospkg, signal, sys
from geometry_msgs.msg import WrenchStamped
import numpy as np


MEAN_WINDOW = 30
wrench_fifo = np.empty([1, 6])

wrench_filtered = None


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
    mean_wrench = np.mean(wrench_fifo, axis = 0)

    # Publish filter wrench value
    filtered = WrenchStamped()
    filtered.header.frame_id = "base_link"
    filtered.header.stamp = rospy.Time()
    filtered.wrench.force.x = mean_wrench[0]
    filtered.wrench.force.y = mean_wrench[1]
    filtered.wrench.force.z = mean_wrench[2]
    filtered.wrench.torque.x = mean_wrench[3]
    filtered.wrench.torque.y = mean_wrench[4]
    filtered.wrench.torque.z = mean_wrench[5]

    wrench_filtered.publish(filtered)

    if wrench_fifo.shape[0] > MEAN_WINDOW:
        wrench_fifo = np.delete(wrench_fifo, 0, 0)
         

def main():
    rospy.init_node('wrench_filter', anonymous=True)

    global wrench_filtered
    wrench_filtered = rospy.Publisher('wrench_filtered', WrenchStamped, queue_size=1)

    rospy.Subscriber("wrench", WrenchStamped, wrenchFilter)

    rospy.spin()


if __name__ == '__main__':
    main()