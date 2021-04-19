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
force = np.empty([1, 3])


def signal_handler(sig, frame):
    print('Exiting Program')
    sys.exit(0)


def wrench_filtered(data):
    # Aquisition of the force values
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    global stream, force

    stream.append((f_x, f_y, f_z))
    force = np.append(force, [[f_x, f_y, f_z]], axis=0)
         

def main():
    rospy.init_node('wrench_record', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    global arm, stream, force

    arm = Arm('ur10e_moveit', group='manipulator', joint_positions_filename="positions.yaml")
    arm.velocity = 1

    # rospy.Subscriber("wrench", WrenchStamped, wrench_filtered)
    
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
    for w_1 in [-180]:
        for w_2 in [0]:
            if (w_1, w_2) not in forbidden:
                # Set Arm joints
                arm.move_joints([0, radians(-90), 0, radians(w_1), radians(w_2), radians(0)])

                # Reset ft sensor
                helpers.reset_ft_sensor()

                # Init temp stream
                temp_stream = []

                # Move wrist 3
                for i in range(-180, 180):
                    print('%d - %d' % (idx, i))
                    arm.move_joints([0, radians(-90), 0, radians(w_1), radians(i), radians(0)])
                    force = np.empty([1, 3])
                    print('Force RESET - %s' % str(force.shape))
                    subs = rospy.Subscriber("wrench_theory", WrenchStamped, wrench_filtered)
                    while force.shape[0] < 5:
                        continue
                    subs.unregister()
                    print('Force Shape - %s' % str(force.shape))
                    # temp_stream.append((statistics.mean(force[:,0]), statistics.mean(force[:,1]), 
                    #                     statistics.mean(force[:,2])))
                    temp_stream.append((force[-1,0], force[-1,1], force[-1,2]))

                # Save samples of wrench in files
                with open(BASE_DIR + '/record/TTZ%d_%d_%d_temp.list' % (idx, w_1, w_2), 'w') as f:
                    print(len(temp_stream))
                    pickle.dump(temp_stream, f)
                
                # with open(BASE_DIR + '/record/TZ%d_%d_%d_full.list' % (idx, w_1, w_2), 'w') as f:
                #     print(len(stream))
                #     pickle.dump(stream, f)

                idx += 1
                stream = []
        
    # rospy.spin()


if __name__ == '__main__':
    main()