#!/usr/bin/env python

import rospy, rospkg, statistics, time, signal, sys, argparse, pickle
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import WrenchStamped
from math import radians, sqrt

import matplotlib.pyplot as plt
import numpy as np

from menu import callControlArmService
from ArmControl import ArmControl

stream = []

f_x_values = [0]
f_y_values = [0]
f_z_values = [0]

f_x_i_values = []
f_y_i_values = []
f_z_i_values = []

t_x_values = [0]
t_y_values = [0]
t_z_values = [0]

# Live parameters
stream_range = 100
stream_count = 0
integral_range = 20
integral_idx = 0

# Record parameters
force_limit = 15
grip_ready = 500

BASE_DIR = rospkg.RosPack().get_path('ur10e_control')


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
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    # t_x = data.wrench.torque.x
    # t_y = data.wrench.torque.y
    # t_z = data.wrench.torque.z

    # evaluateForce(x, y, z)

    global stream, stream_count, integral_idx

    # Aquisition of the force values
    stream.append((f_x, f_y, f_z))

    stream_count += 1
    f_x_values.append(f_x)
    f_y_values.append(f_y)
    f_z_values.append(f_z)

    # t_x_values.append(t_x)
    # t_y_values.append(t_y)
    # t_z_values.append(t_z)

    if stream_count > stream_range:
        del f_x_values[0]
        del f_y_values[0]
        del f_z_values[0]

        # del f_x_i_values[0]
        # del f_y_i_values[0]
        # del f_z_i_values[0]

        # del t_x_values[0]
        # del t_y_values[0]
        # del t_z_values[0]


    # # Integration of the force values
    # if integral_idx > integral_range:
    #     # X Mean in integral range
    #     x_i_temp = f_x_values[-1 - integral_range : -1]
    #     x_i_sub = []
    #     for i in range(1, len(x_i_temp)):
    #         x_i_sub.append(x_i_temp[i] - x_i_temp[i-1])
    #     f_x_i_values.append(statistics.mean(x_i_sub))
    #     # Y Mean in integral range
    #     y_i_temp = f_y_values[-1 - integral_range : -1]
    #     y_i_sub = []
    #     for i in range(1, len(y_i_temp)):
    #         y_i_sub.append(y_i_temp[i] - y_i_temp[i-1])
    #     f_y_i_values.append(statistics.mean(y_i_sub))
    #     # z Mean in integral range
    #     z_i_temp = f_z_values[-1 - integral_range : -1]
    #     z_i_sub = []
    #     for i in range(1, len(z_i_temp)):
    #         z_i_sub.append(z_i_temp[i] - z_i_temp[i-1])
    #     f_z_i_values.append(statistics.mean(z_i_sub))
    # else:
    #     integral_idx += 1
         

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
        fig, (f, f_i, t) = plt.subplots(3)

        f.set(ylim=(-10, 10))
        f.set(xlim=(0, stream_range))

        f_i.set(ylim=(-1, 1))
        f_i.set(xlim=(0, stream_range))

        t.set(ylim=(-1, 1))
        t.set(xlim=(0, stream_range))

        x_plt, = f.plot(range(0, len(f_x_values)), f_x_values, 'r-')
        y_plt, = f.plot(range(0, len(f_y_values)), f_y_values, 'g-')
        z_plt, = f.plot(range(0, len(f_z_values)), f_z_values, 'b-')

        x_i_plt, = f_i.plot(range(0, len(f_x_i_values)), f_x_i_values, 'r-')
        y_i_plt, = f_i.plot(range(0, len(f_y_i_values)), f_y_i_values, 'g-')
        z_i_plt, = f_i.plot(range(0, len(f_z_i_values)), f_z_i_values, 'b-')

        t_x_plt, = t.plot(range(0, len(t_x_values)), t_x_values, 'r-')
        t_y_plt, = t.plot(range(0, len(t_y_values)), t_y_values, 'g-')
        t_z_plt, = t.plot(range(0, len(t_z_values)), t_z_values, 'b-')

        while True:
            setPlotData([x_plt, y_plt, z_plt], [f_x_values[:], f_y_values[:], f_z_values[:]])
            setPlotData([x_i_plt, y_i_plt, z_i_plt], [f_x_i_values[:], f_y_i_values[:], f_z_i_values[:]])
            setPlotData([t_x_plt, t_y_plt, t_z_plt], [t_x_values[:], t_y_values[:], t_z_values[:]])

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
    
    else:
        arm = ArmControl()
        arm.setSpeed(0.3)

        # Reset wrist_3
        current_joints = arm.getJointValues()
        current_joints[5] = radians(0)
        arm.jointGoal(current_joints)

        # Reset ft sensor
        rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor')
        try:
            zero = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)
            resp1 = zero()
            print(resp1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # Init temp stream
        temp_stream = []

        # Move wrist 3
        for i in range(-180, 180):
            print(i, ' - ', len(stream))
            current_joints[5] = radians(i)
            arm.jointGoal(current_joints)
            time.sleep(0.2)
            temp_stream.append((statistics.mean(f_x_values), statistics.mean(f_y_values), statistics.mean(f_z_values)))
            print('')

        with open(BASE_DIR + '/record/T5_temp.list', 'w') as f:
            print(len(temp_stream))
            pickle.dump(temp_stream, f)
        
        with open(BASE_DIR + '/record/T5_full.list', 'w') as f:
            print(len(stream))
            pickle.dump(stream, f)
        
        
        plt.ylim([-7.5, 15])
        plt.plot(range(-180, 180), [x[0] for x in temp_stream], 'r')
        plt.plot(range(-180, 180), [x[1] for x in temp_stream], 'g')
        plt.plot(range(-180, 180), [x[2] for x in temp_stream], 'b')

        plt.show()
        
    rospy.spin()


if __name__ == '__main__':
    main()