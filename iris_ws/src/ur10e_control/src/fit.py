#!/usr/bin/env python
import rospy, rospkg, pickle

from math import radians
import matplotlib.pyplot as plt
import numpy as np

from dynamic_reconfigure.server import Server
from ur10e_control.cfg import FitConfig

BASE_DIR = rospkg.RosPack().get_path('ur10e_control')

def resetTest(x, plt):
    stream1 = []
    with open(BASE_DIR + '/record/T5_temp.list') as f:
        stream1 = pickle.load(f)

    plt.plot(x, [t1[0] for t1 in stream1], 'r')
    plt.plot(x, [t1[1] for t1 in stream1], 'g')
    plt.plot(x, [t1[2] for t1 in stream1], 'b')

    stream2 = []
    with open(BASE_DIR + '/record/T6_temp.list') as f:
        stream2 = pickle.load(f)

    plt.plot(x, [t2[0] for t2 in stream2], 'r--')
    plt.plot(x, [t2[1] for t2 in stream2], 'g--')
    plt.plot(x, [t2[2] for t2 in stream2], 'b--')
    
    comp_value = stream1[len(stream1)/2 - 75]
    stream3 = []
    for i in range(len(stream2)):
        stream3.append((stream2[i][0] + comp_value[0], stream2[i][1] + comp_value[1], stream2[i][2] + comp_value[2]))

    plt.plot(x, [t3[0] for t3 in stream3], 'r:')
    plt.plot(x, [t3[1] for t3 in stream3], 'g:')
    plt.plot(x, [t3[2] for t3 in stream3], 'b:')
    

def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {x}, {y}, {amp}, {period}""".format(**config))
    return config

def main():
    rospy.init_node("fit", anonymous = False)

    srv = Server(FitConfig, callback)

    # Results file name
    results = [
        '/record/1-13_01/T1_temp.list',
        '/record/1-13_01/T2_temp.list',
        '/record/1-13_01/T3_temp.list',
        '/record/1-13_01/T4_temp.list',
        '/record/1-13_01/T5_temp.list',
        '/record/T1_temp.list',
        '/record/T2_temp.list',
        '/record/T3_temp.list',
        '/record/T4_temp.list',
        '/record/T5_temp.list'
    ]

    # Attempt of a sin function to fit results
    x_global = np.arange(-180,180,1)
    x = 4   * np.sin(np.radians(2.1*(x_global - 60))) + 2
    y = 1.5 * np.sin(np.radians(2*(x_global + 40))) - 0.5
    z = 2   * np.sin(np.radians(2*(x_global + 40))) - 2.5

    plt.ylim([-10, 15.0])

    # Show multiple results from wrench records
    # for result in results:
    #     stream = []
    #     with open(BASE_DIR + result) as f:
    #         stream = pickle.load(f)

    #     plt.plot(x_global, [t[0] for t in stream], 'r:')
    #     plt.plot(x_global, [t[1] for t in stream], 'g:')
    #     plt.plot(x_global, [t[2] for t in stream], 'b:')

    # Reset in different angles test function
    resetTest(x_global, plt)

    # plt.plot(x_global, x, 'r:')
    # plt.plot(x_global, y, 'g:')
    # plt.plot(x_global, z, 'b:')

    plt.show()

if __name__ == "__main__":
    main()