#!/usr/bin/env python
import rospy, rospkg, statistics, time, signal, sys, argparse, pickle
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import WrenchStamped
from math import radians, degrees, sqrt
import matplotlib.pyplot as plt
import numpy as np

from helpers import reset_ft_sensor, set_speed_slider
from sami.arm import Arm

arm = None
correction = None

stream = []
force = np.empty([1, 3])

# Live parameters
stream_range = 50
stream_count = 0

# Record parameters
force_limit = 15
grip_ready = 500

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')


def signal_handler(sig, frame):
    print('')
    sys.exit(0)


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
         

def main():
    rospy.init_node('wrench_listener', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    global arm, correction, stream, stream_count

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

        f.set(ylim=(-15, 15))
        f.set(xlim=(0, stream_range))

        x_plt, = f.plot(range(0, len(force[:, 0])), force[:, 0], 'r')
        y_plt, = f.plot(range(0, len(force[:, 1])), force[:, 1], 'g')
        z_plt, = f.plot(range(0, len(force[:, 2])), force[:, 2], 'b')

        while True:
            setPlotData([x_plt, y_plt, z_plt], [force[:,0], force[:,1], force[:,2]])
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
    
    else:
        positions = [
            # [0, radians(-90), 0, 0, radians(90), 0],
            # [0, radians(-90), 0, 0, radians(45), 0],
            [0, radians(-90), radians(-90), 0, 0, 0]
            # [0, radians(-90), 0, 0, radians(-45), 0],
            # [0, radians(-90), 0, 0, radians(-90), 0],
        ]

        for pos in positions:
            # Set Arm joints
            arm.move_joints(pos)

            # Reset ft sensor
            reset_ft_sensor()

            # Init temp stream
            temp_stream = []

            # Move wrist 3
            for i in range(-180, 180):
                print(i, ' - ', len(stream))
                pos[5] = radians(i)
                arm.move_joints(pos)
                time.sleep(1)
                temp_stream.append((statistics.mean(force[:,0]), statistics.mean(force[:,1]), 
                                    statistics.mean(force[:,2])))

            # Save samples of wrench in files
            with open(BASE_DIR + '/record/TG3_theory_temp.list', 'w') as f:
                print(len(temp_stream))
                pickle.dump(temp_stream, f)
            
            with open(BASE_DIR + '/record/TG3_theory_full.list', 'w') as f:
                print(len(stream))
                pickle.dump(stream, f)

            stream = []
            stream_count = 0
        
        # Plot results
        plt.ylim([-7.5, 15])
        plt.plot(range(-180, 180), [x[0] for x in temp_stream], 'r')
        plt.plot(range(-180, 180), [x[1] for x in temp_stream], 'g')
        plt.plot(range(-180, 180), [x[2] for x in temp_stream], 'b')

        plt.show()
        
    rospy.spin()


if __name__ == '__main__':
    main()