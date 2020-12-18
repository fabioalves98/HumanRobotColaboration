#!/usr/bin/env python

import sys, time, csv
import rospy, rospkg, rosparam, tf
import argparse
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ast import literal_eval
from math import pi, cos, sin

from ur10e_control.msg import ArmCommand
from ur10e_control.srv import ControlArm
from moveit_msgs.msg import PlanningSceneComponents, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Point, TransformStamped, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from ArmControl import ArmControl

arm = None

BASE_DIR = rospkg.RosPack().get_path('ur10e_control')

positions_file = "positions.yaml"
positions = {}
position_names = []


def load_positions(path):
    '''Returns tuple with keys of all possible positions and the dict with the positions previously
       saved in a yaml file'''

    data = rosparam.load_file(path)
    k = data[0][0].keys()

    return k, data[0][0]


def parseRotationArgs(args):
    for i in range(0, len(args)):
        args[i] = eval(args[i].replace('pi', str(pi)))
    return args


def commandStr():
    commands = "\nAvailable commands:\n \
    \tspeed  <0 - 1>     -> Set robot speed\n \
    \tmove   <x> <y> <z> -> Simple cartesian movement relative to last position\n \
    \trotate <x> <y> <z> -> Simple rotation relative to last position\n \
    \tgrip               -> Close the gripper fingers\n \
    \trelease            -> Release the gripper\n \
    \t<position_name>    -> Joint goal to a <position_name>. Names from positions.yaml\n \
    \tsave   <pos_name>  -> Save the current joint values of the arm\n"
    
    return commands


def takeCommandService(req):
    output = parseParams(req.command)
    if output == -1:
        return [False, 'An error occured!']
    return [True, output]


def parseParams(args):
    global positions, position_names, positions_file
    print("\nParsing Command")
    try:
        print (args)
        command = args[0]
        # X > 0 - Forward
        # y > 0 - Left
        # Z > 0 - Upwards
        if("speed" in command):
            return "Current Speed - " + str(arm.setSpeed(float(args[1])))
        elif("move" in command):
            arm.simpleMove([float(args[1]), float(args[2]), float(args[3])], pi/4)
        elif("rotate" in command):
            args = parseRotationArgs(args[1:4])
            arm.simpleRotate([args[0], args[1], args[2]])
        elif(command in position_names):
            arm.jointGoal(positions[command])
            return 'Moving to ' + command
        elif("grip" in command):
            arm.grip()
        elif("release" in command):
            arm.release()
        elif("save" in command):
            pos_name = args[1]
            arm.saveJointPosition(BASE_DIR + "/yaml/" + positions_file, pos_name)
            position_names, positions = load_positions(BASE_DIR + "/yaml/" + positions_file)
        else:
            return commandStr()

    except Exception as e:
        if len(args) == 0:
            test()
        else:
            rospy.logerr("[CORK-IRIS] An error occured while parsing the arguments:")
            rospy.logerr(e)
            rospy.logerr("Check the help command for more information.")   
            return -1


def test():
    pass    
    # arm.setSpeed(0.1)
    # p = arm.getPose().position
    # o = arm.getPose().orientation
    # arm.poseGoal([p.x, p.y, p.z], [-0.471886915591, 0.0268562771098, 0.859489799629, -0.194624544454])

    # arm.jointGoal(positions['vert_pick_pos'])
    # arm.saveJointPosition(BASE_DIR + "/yaml/positions.yaml", "init_calibration_pos")

    # rospy.spin()


def main():
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('controller', anonymous=False)

    global arm, positions, position_names, positions_file

    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", help="run controller in simulation mode or in real mode", action="store_true")
    parser.add_argument("--pos", help="run controller in simulation mode or in real mode", type=str)
    args = parser.parse_args()

    if args.pos:
        positions_file = args.pos

    rospy.loginfo("Using " + positions_file + " file")
    position_names, positions = load_positions(BASE_DIR + "/yaml/" + positions_file)

    if args.sim:
        rospy.loginfo("Connecting to simulation")
        arm = ArmControl('localhost')
    else:
        rospy.loginfo("Connecting to robot")
        arm = ArmControl()
        arm.setSpeed(0.3)
        # arm.config_gripper(100.0)

    arm.printGeneralStatus()

    cmd_receiver = rospy.Service('/cork_iris/control_arm', ControlArm, takeCommandService)
    
    rospy.spin()

if __name__ == "__main__":
    main()