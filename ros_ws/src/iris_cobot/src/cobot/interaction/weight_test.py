#!/usr/bin/env python
import rospy
import time
import pickle
import numpy as np

from geometry_msgs.msg import WrenchStamped

import cobot.helpers as helpers

MEAN_WINDOW = 500
NUM_TESTS = 10
wrench_fifo = np.empty([1, 6])


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
    # mean_wrench = np.mean(wrench_fifo, axis = 0)

    if wrench_fifo.shape[0] > MEAN_WINDOW:
        wrench_fifo = np.delete(wrench_fifo, 0, 0)
    

def main():
    rospy.init_node('weight_test', anonymous=True)

    rospy.Subscriber("wrench", WrenchStamped, wrenchFilter)

    global wrench_fifo

    test_name = 'z_neg_weight'

    tests = []
    for i in range(NUM_TESTS):
        raw_input("\nStart Test? %d" % i)
        helpers.reset_ft_sensor()
        time.sleep(0.5)

        helpers.samiGripService()
        time.sleep(1)
        wrench_fifo = np.empty([1, 6])
        print(wrench_fifo.shape)
        while wrench_fifo.shape != (500, 6):
            pass
        
        print(wrench_fifo.shape)
        mean = np.mean(wrench_fifo, axis = 0)
        print([round(i, 3) for i in mean])
        tests.append(mean)

        print(np.linalg.norm(mean[:3]))
        raw_input("End Test? %d" % i)
        helpers.samiReleaseService()

    for test in tests:
        print([round(i, 3) for i in test])

    with open(helpers.BASE_DIR + '/record/' + test_name, 'w') as f:
        pickle.dump(tests, f)



if __name__ == '__main__':
    main()
