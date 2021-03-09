#!/usr/bin/env python
import rospy, rospkg, statistics, time, signal, sys, argparse, pickle
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import WrenchStamped
from math import radians, degrees, sqrt
import matplotlib.pyplot as plt
import numpy as np

from helpers import reset_ft_sensor, set_speed_slider
from sami.arm import Arm # pylint: disable=import-error, no-name-in-module

arm = None
correction = None

stream = []
force = np.empty([1, 3])

# Live parameters
stream_range = 100
stream_count = 0

# Record parameters
force_limit = 15
grip_ready = 500

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')


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
            print('Release')
            grip_ready = 500

        if grip:
            print('Grip')
            grip_ready = 500


def setPlotData(plts, data):
    if not all(elem == len(data[0]) for elem in [len(x) for x in data]):
        return

    for i in range(len(plts)):
        plts[i].set_xdata(range(0, len(data[i])))
        plts[i].set_ydata(data[i])


def wrench(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    global stream, stream_count, force

    # Aquisition of the force values
    stream.append((f_x, f_y, f_z))
    force = np.append(force, [[f_x, f_y, f_z]], axis=0)
    stream_count += 1

    if stream_count > stream_range:
        force = np.delete(force, 0, 0)


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

    global arm, correction

    with open(BASE_DIR + '/curves/wrench_correct_mean.list') as f:
        correction = pickle.load(f)

    arm = Arm('ur10e_moveit', group='manipulator', joint_positions_filename="positions.yaml")
    arm.velocity = 0.2

    parser = argparse.ArgumentParser()
    parser.add_argument("--live", help="run listener with a live view of the values", action="store_true")
    args = parser.parse_args()

    rospy.Subscriber("wrench", WrenchStamped, wrench)

    if args.live:
        # Create Plots
        plt.ion()
        fig, (f) = plt.subplots(1)

        f.set(ylim=(-10, 10))
        f.set(xlim=(0, stream_range))

        x_plt, = f.plot(range(0, len(force[:, 0])), force[:, 0], 'r')
        y_plt, = f.plot(range(0, len(force[:, 1])), force[:, 1], 'g')
        z_plt, = f.plot(range(0, len(force[:, 2])), force[:, 2], 'b')

        while True:
            setPlotData([x_plt, y_plt, z_plt], [force[:,0], force[:,1], force[:,2]])
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
    
    else:
        # Set Arm joints
        current_joints = arm.get_joints()
        current_joints[0] = radians(0)
        current_joints[1] = radians(-90)
        current_joints[2] = radians(0)
        current_joints[3] = radians(0)
        current_joints[4] = radians(-90)
        current_joints[5] = radians(0)
        arm.move_joints(current_joints)

        # Reset ft sensor
        reset_ft_sensor()

        # Init temp stream
        temp_stream = []

        # Move wrist 3
        for i in range(-180, 180):
            print(i, ' - ', len(stream))
            current_joints[5] = radians(i)
            arm.move_joints(current_joints)
            time.sleep(0.2)
            temp_stream.append((statistics.mean(force[:,0]), statistics.mean(force[:,1]), statistics.mean(force[:,2])))
            print('')

        # Save samples of wrench in files
        with open(BASE_DIR + '/record/TG5_3Kg_temp.list', 'w') as f:
            print(len(temp_stream))
            pickle.dump(temp_stream, f)
        
        with open(BASE_DIR + '/record/TG5_3Kg_full.list', 'w') as f:
            print(len(stream))
            pickle.dump(stream, f)
        
        # Plot results
        plt.ylim([-7.5, 15])
        plt.plot(range(-180, 180), [x[0] for x in temp_stream], 'r')
        plt.plot(range(-180, 180), [x[1] for x in temp_stream], 'g')
        plt.plot(range(-180, 180), [x[2] for x in temp_stream], 'b')

        plt.show()
        
    rospy.spin()


if __name__ == '__main__':
    main()