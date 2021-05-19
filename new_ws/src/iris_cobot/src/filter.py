#!/usr/bin/env python
import rospy, rospkg, signal, sys
from geometry_msgs.msg import WrenchStamped
import numpy as np


MEAN_WINDOW = 30
force_fifo = np.empty([1, 3])

wrench_filtered = None


def wrench(data):
    # Aquisition of the force values
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    # Filter force values
    global force_fifo
    force_fifo = np.append(force_fifo, [[f_x, f_y, f_z]], axis=0)
    mean_force = np.mean(force_fifo, axis = 0)

    # Publish filter force value
    filtered = WrenchStamped()
    filtered.header.frame_id = "base_link"
    filtered.header.stamp = rospy.Time()
    filtered.wrench.force.x = mean_force[0]
    filtered.wrench.force.y = mean_force[1]
    filtered.wrench.force.z = mean_force[2]

    wrench_filtered.publish(filtered)

    if force_fifo.shape[0] > MEAN_WINDOW:
        force_fifo = np.delete(force_fifo, 0, 0)
         

def main():
    rospy.init_node('wrench_filter', anonymous=True)

    global wrench_filtered
    wrench_filtered = rospy.Publisher('wrench_filtered', WrenchStamped, queue_size=1)

    rospy.Subscriber("wrench", WrenchStamped, wrench)

    rospy.spin()


if __name__ == '__main__':
    main()