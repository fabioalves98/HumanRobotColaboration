#!/usr/bin/env python

import rospy, statistics, time, signal, sys, argparse
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped

import matplotlib.pyplot as plt
import numpy as np

stream = []
full_stream = []

x_values = []
y_values = []
z_values = []


def signal_handler(sig, frame):
    print('')
    sys.exit(0)


def callback(data):
    x = data.wrench.force.x
    y = data.wrench.force.y
    z = data.wrench.force.z

    global stream, full_stream

    stream.append((x, y, z))
    full_stream.append((x, y, z))
    x_values.append(x)
    y_values.append(y)
    z_values.append(z)
    
    if len(full_stream) > 5000:
        del x_values[0]
        del y_values[0]
        del z_values[0]

    # if len(stream) > 100:
    #     to_print = '\nMin Max' + '\n'
    #     to_print += 'X - ' + str(min([e[0] for e in stream])) + ' - ' + str(max([e[0] for e in stream])) + '\n'
    #     to_print += 'Y - ' + str(min([e[1] for e in stream])) + ' - ' + str(max([e[1] for e in stream])) + '\n'
    #     to_print += 'Z - ' + str(min([e[2] for e in stream])) + ' - ' + str(max([e[2] for e in stream])) + '\n'

    #     to_print += 'Mean' + '\n'
    #     to_print += 'X - ' + str(statistics.mean([e[0] for e in stream])) + '\n'
    #     to_print += 'Y - ' + str(statistics.mean([e[1] for e in stream])) + '\n'
    #     to_print += 'Z - ' + str(statistics.mean([e[2] for e in stream])) + '\n'

    #     to_print += 'Std Dev' + '\n'
    #     to_print += 'X - ' + str(statistics.stdev([e[0] for e in stream])) + '\n'
    #     to_print += 'Y - ' + str(statistics.stdev([e[1] for e in stream])) + '\n'
    #     to_print += 'Z - ' + str(statistics.stdev([e[2] for e in stream])) + '\n'

    #     # print(to_print)

    #     stream = []


def main():
    rospy.init_node('wrench_listener', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser()
    parser.add_argument("--live", help="run listener with a live view of the values", action="store_true")
    args = parser.parse_args()

    wrench_sub = rospy.Subscriber("wrench", WrenchStamped, callback)

    if args.live:
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set(ylim=(-50, 50))
        ax.set(xlim=(0, 5000))

        x_plt, = ax.plot(range(0, len(x_values)), x_values, 'r-')
        y_plt, = ax.plot(range(0, len(y_values)), y_values, 'g-')
        z_plt, = ax.plot(range(0, len(z_values)), z_values, 'b-')

        while True:
            x_temp = x_values[:]
            y_temp = y_values[:]
            z_temp = z_values[:]

            if len(x_temp) != len(y_temp) or len(x_temp) != len(z_temp) or len(z_temp) != len(y_temp):
                continue

            x_plt.set_xdata(range(0, len(x_temp)))
            y_plt.set_xdata(range(0, len(y_temp)))
            z_plt.set_xdata(range(0, len(z_temp)))

            x_plt.set_ydata(x_temp)
            y_plt.set_ydata(y_temp)
            z_plt.set_ydata(z_temp)

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
    
    else:
        raw_input("Start Recording?")
        raw_input("Show Graph?")
        wrench_sub.unregister()
        
        plt.plot(range(0, len(full_stream)), [x[0] for x in full_stream], 'r')
        plt.plot(range(0, len(full_stream)), [x[1] for x in full_stream], 'g')
        plt.plot(range(0, len(full_stream)), [x[2] for x in full_stream], 'b')

        plt.show()
        
    rospy.spin()


if __name__ == '__main__':
    main()