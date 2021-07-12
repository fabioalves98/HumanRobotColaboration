#!/usr/bin/env python3
import roboticstoolbox as rtb
from spatialmath import *
from math import radians


ANGLES = [-180, -135, -90, -45, 0, 45, 90, 135]
FORBIDDEN = [(45, -180),  (45, -135),
             (90, -180),  (90, -135), (90, 135),
             (135, -180), (135, 135)]
                        

def main():    
    robot = rtb.models.DH.UR10()

    print(robot.qz)

    T = robot.fkine(robot.qz)  # forward kinematics
    print(T)
    print(type(T))
    print(T.t)
    print(T.R)
    print(type(T.R))
    print(T.eul())
    print(UnitQuaternion(T))


if __name__ == "__main__":
    main()