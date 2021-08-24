#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from roboticstoolbox.robot.ERobot import ERobot
from spatialmath import UnitQuaternion as UQ


ANGLES = [-180, -135, -90, -45, 0, 45, 90, 135]
FORBIDDEN = [(45, -180),  (45, -135),
             (90, -180),  (90, -135), (90, 135),
             (135, -180), (135, 135)]

robot = None


class UR10e(ERobot):
    def __init__(self):

        links, name = self.URDF_read(
            "ur_e_description/urdf/ur10e_robot.urdf.xacro"
        )

        super().__init__(
                links,
                name=name.upper(),
                manufacturer='Universal Robotics'
            )

        self.addconfiguration(
            "qz", np.array([0, 0, 0, 0, 0, 0]))
        self.addconfiguration(
            "qr", np.array([np.pi, 0, 0, 0, np.pi/2, 0]))


def compute_fk(joints):
    pose = robot.fkine(joints, end='ee_link')

    trans = pose.t
    rot = np.append(UQ(pose).vec3, [UQ(pose).s])
    
    pose_msg = Pose()
    pose_msg.position = Point(*trans)
    pose_msg.orientation = Quaternion(*rot)

    return pose_msg


def main():
    rospy.init_node('kinematics', anonymous=False)

    global robot  
    robot = UR10e()
    print(robot)

    rospy.spin()


if __name__ == "__main__":
    main()