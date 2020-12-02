#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped

import statistics, time
import matplotlib.pyplot as plt
import numpy as np


stream = []
full_stream = []

def callback(data):
    # rospy.loginfo('\n' + str(data.wrench))

    x = data.wrench.force.x
    y = data.wrench.force.y
    z = data.wrench.force.z

    global stream, full_stream

    stream.append((x, y, z))
    full_stream.append((x, y, z))

    if len(stream) > 100:
        to_print = '\nMin Max' + '\n'
        to_print += 'X - ' + str(min([e[0] for e in stream])) + ' - ' + str(max([e[0] for e in stream])) + '\n'
        to_print += 'Y - ' + str(min([e[1] for e in stream])) + ' - ' + str(max([e[1] for e in stream])) + '\n'
        to_print += 'Z - ' + str(min([e[2] for e in stream])) + ' - ' + str(max([e[2] for e in stream])) + '\n'

        to_print += 'Mean' + '\n'
        to_print += 'X - ' + str(statistics.mean([e[0] for e in stream])) + '\n'
        to_print += 'Y - ' + str(statistics.mean([e[1] for e in stream])) + '\n'
        to_print += 'Z - ' + str(statistics.mean([e[2] for e in stream])) + '\n'

        to_print += 'Std Dev' + '\n'
        to_print += 'X - ' + str(statistics.stdev([e[0] for e in stream])) + '\n'
        to_print += 'Y - ' + str(statistics.stdev([e[1] for e in stream])) + '\n'
        to_print += 'Z - ' + str(statistics.stdev([e[2] for e in stream])) + '\n'

        # print(to_print)

        stream = []


def main():
    rospy.init_node('wrench_listener', anonymous=True)
    
    raw_input("Start Recording?")
    
    wrench_sub = rospy.Subscriber("wrench", WrenchStamped, callback)

    raw_input("Show Graph?")

    wrench_sub.unregister()

    # Show Graph
    print(len(full_stream))
    
    static_plot = True
    if (static_plot):
        plt.plot(range(0, len(full_stream)), [x[0] for x in full_stream], 'r')
        plt.plot(range(0, len(full_stream)), [x[1] for x in full_stream], 'g')
        plt.plot(range(0, len(full_stream)), [x[2] for x in full_stream], 'b')

        plt.show()
    else:
        y = [x[0] for x in full_stream]

        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)

        for idx in range(1, len(y) - 100):
            line, = ax.plot(range(0, 100), y[idx:100+idx], 'b-')
            # line.set_ydata(y[idx:100+idx])
            line.set
            fig.clear()
            fig.canvas.draw()

    rospy.spin()

if __name__ == '__main__':
    main()