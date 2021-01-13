#!/usr/bin/env python
import rospy, rospkg, pickle

from math import radians
import matplotlib.pyplot as plt
import numpy as np

from dynamic_reconfigure.server import Server
from ur10e_control.cfg import FitConfig

BASE_DIR = rospkg.RosPack().get_path('ur10e_control')

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {x}, {y}, {amp}, {period}""".format(**config))
    return config

def main():
    rospy.init_node("fit", anonymous = False)

    srv = Server(FitConfig, callback)

    stream = []

    with open(BASE_DIR + '/record/T1_temp.list') as f:
        stream = pickle.load(f)

    x_global = np.arange(-180,180,1)
    x = 4   * np.sin(np.radians(2.1*(x_global - 60))) + 2
    y = 1.5 * np.sin(np.radians(2*(x_global + 40))) - 0.5
    z = 2   * np.sin(np.radians(2*(x_global + 40))) - 2.5

    plt.ylim([-7.5, 15.0])

    plt.plot(x_global, [t[0] for t in stream], 'r')
    plt.plot(x_global, [t[1] for t in stream], 'g')
    plt.plot(x_global, [t[2] for t in stream], 'b')

    plt.plot(x_global, x, 'r:')
    plt.plot(x_global, y, 'g:')
    plt.plot(x_global, z, 'b:')
    plt.show()

if __name__ == "__main__":
    main()