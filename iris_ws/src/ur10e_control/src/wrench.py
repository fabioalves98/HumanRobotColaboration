#!/usr/bin/env python

import rospy, statistics, time, signal, sys, argparse
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped

import matplotlib.pyplot as plt
import numpy as np

from menu import callControlArmService

stream = []
full_stream = []

x_values = [0]
y_values = [0]
z_values = [0]

x_i_values = []
y_i_values = []
z_i_values = []

stream_range = 2000
stream_count = 0
integral_range = 20
integral_idx = 0

force_limit = 15
grip_ready = 500


def signal_handler(sig, frame):
    print('')
    sys.exit(0)


def evaluateForce(x, y, z):
    global grip_ready
    grip_ready -= 1
    release, grip = False, False
    if abs(x) > force_limit:
        grip = True
    if abs(y) > force_limit or z > force_limit:
        release = True
    
    if grip_ready < 0:
        if release:
            callControlArmService(['release'])
            print('Release')
            grip_ready = 500

        if grip:
            callControlArmService(['grip'])
            print('Grip')
            grip_ready = 500


def setPlotData(plts, data):
    if not all(elem == len(data[0]) for elem in [len(x) for x in data]):
        return

    for i in range(len(plts)):
        plts[i].set_xdata(range(0, len(data[i])))
        plts[i].set_ydata(data[i])


def callback(data):
    x = data.wrench.force.x
    y = data.wrench.force.y
    z = data.wrench.force.z

    evaluateForce(x, y, z)

    global stream, stream_count, full_stream, integral_idx

    # Aquisition of the force values
    stream.append((x, y, z))
    # full_stream.append((x, y, z))

    stream_count += 1
    x_values.append(x)
    y_values.append(y)
    z_values.append(z)

    if stream_count > stream_range:
        del x_values[0]
        del y_values[0]
        del z_values[0]

        del x_i_values[0]
        del y_i_values[0]
        del z_i_values[0]

    # Integration of the force values
    if integral_idx > integral_range:
        # X Mean in integral range
        x_i_temp = x_values[-1 - integral_range : -1]
        x_i_sub = []
        for i in range(1, len(x_i_temp)):
            x_i_sub.append(x_i_temp[i] - x_i_temp[i-1])
        x_i_values.append(statistics.mean(x_i_sub))
        # Y Mean in integral range
        y_i_temp = y_values[-1 - integral_range : -1]
        y_i_sub = []
        for i in range(1, len(y_i_temp)):
            y_i_sub.append(y_i_temp[i] - y_i_temp[i-1])
        y_i_values.append(statistics.mean(y_i_sub))
        # z Mean in integral range
        z_i_temp = z_values[-1 - integral_range : -1]
        z_i_sub = []
        for i in range(1, len(z_i_temp)):
            z_i_sub.append(z_i_temp[i] - z_i_temp[i-1])
        z_i_values.append(statistics.mean(z_i_sub))
    else:
        integral_idx += 1
         

def main():
    rospy.init_node('wrench_listener', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser()
    parser.add_argument("--live", help="run listener with a live view of the values", action="store_true")
    args = parser.parse_args()

    wrench_sub = rospy.Subscriber("wrench", WrenchStamped, callback)

    if args.live:
        # Create Plots
        plt.ion()
        fig, (ax, ax_i) = plt.subplots(2)

        ax.set(ylim=(-30, 30))
        ax.set(xlim=(0, stream_range))

        ax_i.set(ylim=(-1, 1))
        ax_i.set(xlim=(0, stream_range))

        x_plt, = ax.plot(range(0, len(x_values)), x_values, 'r-')
        y_plt, = ax.plot(range(0, len(y_values)), y_values, 'g-')
        z_plt, = ax.plot(range(0, len(z_values)), z_values, 'b-')

        x_i_plt, = ax_i.plot(range(0, len(x_i_values)), x_i_values, 'r-')
        y_i_plt, = ax_i.plot(range(0, len(y_i_values)), y_i_values, 'g-')
        z_i_plt, = ax_i.plot(range(0, len(z_i_values)), z_i_values, 'b-')

        while True:
            setPlotData([x_plt, y_plt, z_plt], [x_values[:], y_values[:], z_values[:]])
            setPlotData([x_i_plt, y_i_plt, z_i_plt], [x_i_values[:], y_i_values[:], z_i_values[:]])

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