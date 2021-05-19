#!/usr/bin/env python
import rospy, rospkg
import pickle
from math import degrees
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import helpers

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')

wrench_pub = None
# Positional Correction
correction = None
index = 0
# Orientational Correction
theory_values = (0, 0, 0)
theory_offset = (0, 0, 0)


def ftResetService(req):
    global index, correction, theory_offset
    offset = correction[index,:]

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
    index = int(degrees(w3_joint)) + 180


def wrenchTheory(data):
    t_x = data.wrench.force.x
    t_y = data.wrench.force.y
    t_z = data.wrench.force.z

    global theory_values
    theory_values = (t_x, t_y, t_z)

    
def wrenchCorrect(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    global index
    force = (f_x, f_y, f_z)

    corrected = WrenchStamped()
    corrected.header.frame_id = "base_link"
    corrected.header.stamp = rospy.Time()
    comp = correction[index,:]
    corrected.wrench.force.x = force[0] - comp[0] - (theory_values[0] - theory_offset[0])
    corrected.wrench.force.y = force[1] - comp[1] - (theory_values[1] - theory_offset[1])
    corrected.wrench.force.z = force[2] - comp[2] - (theory_values[2] - theory_offset[2])

    wrench_pub.publish(corrected)
    

def main():
    rospy.init_node('correct', anonymous=True)

    global correction, wrench_pub

    with open(BASE_DIR + '/curves/wrench_correct_56.list') as f:
        correction = pickle.load(f).transpose()
        print(correction.shape)

    rospy.wait_for_message('joint_states', JointState)
    joints_sub = rospy.Subscriber('joint_states', JointState, jointUpdate)
    rospy.wait_for_message('wrench_theory', WrenchStamped)
    theory_sub = rospy.Subscriber('wrench_theory', WrenchStamped, wrenchTheory)
    rospy.wait_for_message('wrench_theory', WrenchStamped)
    
    
    rospy.Service('/iris_cobot/zero_ftsensor', Trigger, ftResetService)
    ftResetService(None)

    wrench_pub = rospy.Publisher('wrench_correct', WrenchStamped, queue_size=1)
    rospy.wait_for_message('wrench_filtered', WrenchStamped)
    wrench_sub = rospy.Subscriber('wrench_filtered', WrenchStamped, wrenchCorrect)


    rospy.spin()


if __name__ == '__main__':
    main()