#!/usr/bin/env python
import rospy

from math import radians
import matplotlib.pyplot as plt
import numpy as np


def main():
    x_global = np.arange(-180,180,1)
    x = 4 * np.sin(np.radians(2*(x_global - 60))) + 2
    y = 1 * np.sin(np.radians(2*(x_global + 50))) - 1
    z = 2 * np.sin(np.radians(2*(x_global + 50))) - 2.5

    plt.ylim([-7.5, 15.0])
    plt.plot(x_global, x, 'r')
    plt.plot(x_global, y, 'g')
    plt.plot(x_global, z, 'b')
    plt.show()

if __name__ == "__main__":
    main()