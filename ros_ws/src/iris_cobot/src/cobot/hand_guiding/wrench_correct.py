#!/usr/bin/env python
import rospy, rospkg
import pickle
import numpy as np
from math import degrees
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import cobot.helpers as helpers

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')

wrench_pub = None
# Positional Correction
correction = None
index = 0
# Orientational Correction
theory_values = np.empty([2, 3])
theory_offset = np.empty([2, 3])


def ftResetService(req):
    global index, correction, theory_offset, theory_values
    offset = correction[index,:,:]

    helpers.reset_ft_sensor()

    theory_offset = theory_values
    correction -= offset

    return [True, "Reset service called"]


def jointUpdate(data):
    w3_joint = 0
    try:
        w3_index = data.name.index('wrist_3_joint')
        w3_joint = data.position[w3_index]
    except ValueError:
        return
    
    global index
    index = int(degrees(w3_joint)) + 360


def wrenchTheory(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    t_x = data.wrench.torque.x
    t_y = data.wrench.torque.y
    t_z = data.wrench.torque.z

    global theory_values
    theory_values = [[f_x, f_y, f_z], [t_x, t_y, t_z]]

    
def wrenchCorrect(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    t_x = data.wrench.torque.x
    t_y = data.wrench.torque.y
    t_z = data.wrench.torque.z

    global index

    corrected = WrenchStamped()
    corrected.header.frame_id = "base_link"
    corrected.header.stamp = rospy.Time()
    comp = correction[index,:,:]
    corrected.wrench.force.x = f_x - comp[0][0] - (theory_values[0][0] - theory_offset[0][0])
    corrected.wrench.force.y = f_y - comp[0][1] - (theory_values[0][1] - theory_offset[0][1])
    corrected.wrench.force.z = f_z - comp[0][2] - (theory_values[0][2] - theory_offset[0][2])
    corrected.wrench.torque.x = t_x - comp[1][0] - (theory_values[1][0] - theory_offset[1][0])
    corrected.wrench.torque.y = t_y - comp[1][1] - (theory_values[1][1] - theory_offset[1][1])
    corrected.wrench.torque.z = t_z - comp[1][2] - (theory_values[1][2] - theory_offset[1][2])

    wrench_pub.publish(corrected)
    

def main():
    rospy.init_node('wrench_correct', anonymous=True)

    global correction, wrench_pub

    with open(BASE_DIR + '/curves/wrench_correct_final_720.list') as f:
        correction = pickle.load(f)
        print(correction.shape)

    joints_sub = rospy.Subscriber('joint_states', JointState, jointUpdate)
    rospy.wait_for_message('joint_states', JointState)

    theory_sub = rospy.Subscriber('wrench_theory', WrenchStamped, wrenchTheory)
    rospy.wait_for_message('wrench_theory', WrenchStamped)
    
    rospy.Service('/iris_cobot/zero_ftsensor', Trigger, ftResetService)
    ftResetService(None)

    wrench_pub = rospy.Publisher('wrench_correct', WrenchStamped, queue_size=1)

    wrench_sub = rospy.Subscriber('wrench_filtered', WrenchStamped, wrenchCorrect)
    rospy.wait_for_message('wrench_filtered', WrenchStamped)

    rospy.INFO("Wrench correct node node listening to wrench_filtered and wrench_theory, and \
                publishing to wrench_filtered")

    rospy.spin()


if __name__ == '__main__':
    main()