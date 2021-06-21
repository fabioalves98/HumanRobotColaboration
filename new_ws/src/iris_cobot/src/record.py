#!/usr/bin/env python
import rospy, rospkg, statistics, time, signal, sys, pickle
import numpy as np
from math import radians
from geometry_msgs.msg import WrenchStamped

import helpers
from sami.arm import Arm

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')

arm = None

stream = []
wrench = np.empty([1, 6])


def signal_handler(sig, frame):
    print('Exiting Program')
    sys.exit(0)


def wrench_filtered(data):
    # Aquisition of the force values
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    t_x = data.wrench.torque.x
    t_y = data.wrench.torque.y
    t_z = data.wrench.torque.z

    global stream, wrench

    # REAL ROBOT RECORD
    # stream.append((f_x, f_y, f_z))
    # wrench = np.append(wrench, [[f_x, f_y, f_z, t_x, t_y, t_z]], axis=0)

    # SIM THEORY RECORD
    wrench = np.array([f_x, f_y, f_z, t_x, t_y, t_z])

         

def main():
    rospy.init_node('wrench_record', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    global arm, stream, wrench

    arm = Arm('ur10e_moveit', group='manipulator', joint_positions_filename="positions.yaml")
    arm.velocity = 1

    # SIM THEORY RECORD
    rospy.Subscriber("wrench_theory", WrenchStamped, wrench_filtered)
    
    # positions = [
    #     # [0, radians(-90), 0, 0, radians(90), 0],
    #     # [0, radians(-90), 0, 0, radians(45), 0],
    #     [0, radians(-90), radians(-90), 0, 0, 0]
    #     # [0, radians(-90), 0, 0, radians(-45), 0],
    #     # [0, radians(-90), 0, 0, radians(-90), 0],
    # ]

    angles = [-180, -135, -90, -45, 0, 45, 90, 135]
    forbidden = [(45, -180),  (45, -135),
                (90, -180),  (90, -135), (90, 135),
                (135, -180),  (135, 135)]
    idx = 0
    idx_skip = 0

    for w_1 in angles:
        for w_2 in angles:
            if (w_1, w_2) not in forbidden:
                # IDX Skip
                if idx < idx_skip:
                    idx += 1
                    continue
                
                # W1 Skip
                # if w_1 not in [0]:
                #     idx += 1
                #     continue

                # Set Arm joints
                arm.move_joints([0, radians(-90), 0, radians(w_1), radians(w_2), radians(0)])

                # Reset ft sensor
                helpers.reset_ft_sensor()

                # Init temp stream
                wrench_sample = []

                # Move wrist 3
                for i in range(-180, 180):
                    print('%d - %d' % (idx, i))
                    arm.move_joints([0, radians(-90), 0, radians(w_1), radians(w_2), radians(i)])
                    # REAL ROBOT RECORD
                    # wrench = np.empty([1, 6])
                    # subs = rospy.Subscriber("wrench_filtered", WrenchStamped, wrench_filtered)
                    # while wrench.shape[0] < 30:
                    #     continue
                    # subs.unregister()
                    # wrench_sample.append([(statistics.mean(wrench[:,0]), 
                    #                        statistics.mean(wrench[:,1]), 
                    #                        statistics.mean(wrench[:,2])),
                    #                       (statistics.mean(wrench[:,3]), 
                    #                        statistics.mean(wrench[:,4]), 
                    #                        statistics.mean(wrench[:,5]))])

                    # SIM THEORY RECORD
                    wrench_sample.append([(wrench[0], wrench[1], wrench[2]),
                                          (wrench[3], wrench[4], wrench[5])])

                # Save samples of wrench in files
                with open(BASE_DIR + '/record/TCWT%d_%d_%d.list' % (idx, w_1, w_2), 'w') as f:
                    print(len(wrench_sample))
                    pickle.dump(wrench_sample, f)


                idx += 1
                stream = []
        
    # rospy.spin()


if __name__ == '__main__':
    main()