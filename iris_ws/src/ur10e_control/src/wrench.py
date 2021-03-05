#!/usr/bin/env python
import rospy, rospkg
import pickle
from math import degrees
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

BASE_DIR = rospkg.RosPack().get_path('ur10e_control')

wrench_pub = None
correction = None
w3_joint = None


def jointUpdate(data):
    # Get wrist_3 value
    global w3_joint
    try:
        w3_index = data.name.index('wrist_3_joint')
        w3_joint = data.position[w3_index]
    except ValueError:
        pass


def wrenchCorrect(data):
    # Get wrist_3 index
    index = int(degrees(w3_joint)) + 180

    # Access Correction Array
    comp = correction[:, index]

    # Correct
    data.wrench.force.x -= comp[0]
    data.wrench.force.y -= comp[1]
    data.wrench.force.z -= comp[2]

    print(index, comp, data.wrench.force)

    # Publish Corrected Wrench
    wrench_pub.publish(data)


def main():
    rospy.init_node('wrench_listener', anonymous=True)

    global correction, wrench_pub

    with open(BASE_DIR + '/curves/wrench_correct_mean.list') as f:
        correction = pickle.load(f)

    wrench_sub = rospy.Subscriber('wrench', WrenchStamped, wrenchCorrect)
    joints_sub = rospy.Subscriber('joint_states', JointState, jointUpdate)
    wrench_pub = rospy.Publisher('wrench_correct', WrenchStamped, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()