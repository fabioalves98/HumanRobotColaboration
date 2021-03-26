#!/usr/bin/env python
import rospy, rospkg
import pickle
from math import degrees
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')

wrench_pub = None
correction = None
force = (0, 0, 0)


def jointUpdate(data):
    # Get wrist_3 value
    w3_joint = 0
    try:
        w3_index = data.name.index('wrist_3_joint')
        w3_joint = data.position[w3_index]
    except ValueError:
        pass
    
    index = int(degrees(w3_joint)) + 180
    comp = correction[:, index]

    print('\n%d\nComp - %s' % (index, comp))

    global force
    corrected = WrenchStamped()
    corrected.header.frame_id = "base_link"
    corrected.header.stamp = rospy.Time()
    corrected.wrench.force.x = force[0] - comp[0]
    corrected.wrench.force.y = force[1] - comp[1]
    corrected.wrench.force.z = force[2] - comp[2]

    wrench_pub.publish(corrected)


def wrenchCorrect(data):
    f_x = data.wrench.force.x
    f_y = data.wrench.force.y
    f_z = data.wrench.force.z

    global force
    force = (f_x, f_y, f_z)

    # # Get wrist_3 index
    # index = int(degrees(w3_joint)) + 180

    # # Access Correction Array
    # comp = correction[:, index]

    # # Correct
    # data.wrench.force.x -= comp[0]
    # data.wrench.force.y -= comp[1]
    # data.wrench.force.z -= comp[2]

    # print(index, comp, data.wrench.force)

    # # Publish Corrected Wrench
    # wrench_pub.publish(data)


def main():
    rospy.init_node('correct', anonymous=True)

    global correction, wrench_pub

    with open(BASE_DIR + '/curves/wrench_correct_mean.list') as f:
        correction = pickle.load(f)
        print(correction.shape)

    wrench_sub = rospy.Subscriber('wrench', WrenchStamped, wrenchCorrect)
    joints_sub = rospy.Subscriber('joint_states', JointState, jointUpdate)
    wrench_pub = rospy.Publisher('wrench_correct', WrenchStamped, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()