#!/usr/bin/env python
import rospy, rospkg, pickle
import numpy as np
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from ur_msgs.srv import SetSpeedSliderFraction, SetPayload
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped
from visualization_msgs.msg import Marker
from controller_manager_msgs.srv import ListControllers, SwitchController

from iris_cobot.srv import WeightUpdate
from iris_sami.srv import JointGoalName, JointGoal, RelativeMove, PoseGoal, NoArguments

BASE_DIR = rospkg.RosPack().get_path('iris_cobot')

ANGLES = [-180, -135, -90, -45, 0, 45, 90, 135]
FORBIDDEN = [(45, -180),  (45, -135),
             (90, -180),  (90, -135), (90, 135),
             (135, -180), (135, 135)]


def set_speed_slider(speed):
    try:
        rospy.wait_for_service('/ur_hardware_interface/set_speed_slider', timeout=2)
        set_speed = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
        resp = set_speed(speed)
        return resp.success
    except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
        print("Service call failed: %s"%e)


def reset_ft_sensor():
    try:
        rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor', timeout=2)
        zero = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)
        resp = zero()
        return resp.success
    except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
        print("Service call failed: %s"%e)


def cobot_reset_ft_sensor():
    try:
        rospy.wait_for_service('/iris_cobot/zero_ftsensor', timeout=2)
        zero = rospy.ServiceProxy('/iris_cobot/zero_ftsensor', Trigger)
        resp = zero()
        return resp.success
    except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
        print("Service call failed: %s"%e)


def set_payload(mass, cog):
    try:
        rospy.wait_for_service('/ur_hardware_interface/set_payload', timeout=2)
        set_pay = rospy.ServiceProxy('/ur_hardware_interface/set_payload', SetPayload)
        resp = set_pay(mass=mass, center_of_gravity=cog)
        return resp.success
    except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
        print("Service call failed: %s"%e)


def openList(filename):    
    with open(BASE_DIR + filename) as f:
        return np.array(pickle.load(f))


def plotXYZ(plt, x, array, line='', alpha=1, title=''):
    array = np.array(array)

    if array.shape == (3, 360):
        array = array.transpose()

    plt.plot(x, [t[0] for t in array], 'r' + line, alpha=alpha)
    plt.plot(x, [t[1] for t in array], 'g' + line, alpha=alpha)
    plt.plot(x, [t[2] for t in array], 'b' + line, alpha=alpha)
    
    if title:
        plt.set_title(title, loc='center')


def arrowMarker(pose, color):
    ''' Returns an ROS arrow marker
    Parameters:
        pose -          geometry_msgs Pose
        color -         std_msgs ColorRGBA
    '''
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale = Vector3(*[0.2, 0.02, 0.02])
    marker.pose = pose
    marker.color = color

    return marker

def vectorFromQuaternion(quaternion):
    ''' Returns a normalized vector form a quaternion
    Parameters:
        quaterion -     geometry_msgs Quaternion
        color -         std_msgs ColorRGBA
    '''
    initial_pose = PoseStamped(pose=Pose(Point(*[1, 0, 0]), Quaternion(*[0, 0, 0, 1])))

    transform = TransformStamped(transform=Transform(Vector3(*[0, 0, 0]), quaternion))

    new_pose = do_transform_pose(initial_pose, transform)

    return pointToList(new_pose.pose.position)


def pointToList(point):
    p_list = []
    p_list.append(point.x)
    p_list.append(point.y)
    p_list.append(point.z)
    return p_list


def quaternionToList(orientation):
    ori = []
    ori.append(orientation.x)
    ori.append(orientation.y)
    ori.append(orientation.z)
    ori.append(orientation.w)
    return ori


# Update Weight in Theory Model
def weightUpdate(weight):
    try:
        rospy.wait_for_service('/weight_update', timeout=2)
        weighUpdateServ = rospy.ServiceProxy('/weight_update', WeightUpdate)
        resp = weighUpdateServ(weight)
        return resp.success
    except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
        print("Service call failed: %s"%e)


# Switch Controllers
def switchControllers(controller = None):
    controllers = ['joint_group_vel_controller', 'scaled_pos_joint_traj_controller']
    active_controller = ''

    if controller == "position":
        active_controller = controllers[0]
    elif controller == "velocity":
        active_controller = controllers[1]
    elif controller is None:
        # Get active controller
        rospy.wait_for_service('controller_manager/list_controllers')
        try:
            listServ = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
            resp = listServ()
            for controller in resp.controller:
                if controller.name in controllers and controller.state == "running":
                    active_controller = controller.name
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    else:
        print('ERROR IN CONTROLER ARGUMENT PASSING')

    # Switch Controllers
    if active_controller:
        controllers.remove(active_controller)

        rospy.wait_for_service('controller_manager/switch_controller')
        try:
            switchServ = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
            resp = switchServ(start_controllers=controllers,
                            stop_controllers=[active_controller],
                            strictness=1,
                            start_asap=False,
                            timeout=2.0)
            return resp.ok
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    else:
        print('NO CONTROLLERS TO SWITCH')


# SAMI helpers
def samiAliasService(alias):
    rospy.wait_for_service('iris_sami/alias')
    try:
        aliasServ = rospy.ServiceProxy('iris_sami/alias', JointGoalName)
        resp = aliasServ(alias)
        return resp.feedback
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def samiJointService(joints):
    rospy.wait_for_service('iris_sami/joints')
    try:
        jointsServ = rospy.ServiceProxy('iris_sami/joints', JointGoal)
        resp = jointsServ(*joints)
        return resp.feedback
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def samiMoveService(move):
    rospy.wait_for_service('iris_sami/move')
    try:
        moveServ = rospy.ServiceProxy('iris_sami/move', RelativeMove)
        resp = moveServ(*move)
        return resp.feedback
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def samiMoveWorldService(move):
    rospy.wait_for_service('iris_sami/move_world')
    try:
        moveWorldServ = rospy.ServiceProxy('iris_sami/move_world', RelativeMove)
        resp = moveWorldServ(*move)
        return resp.feedback
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def samiPoseService(pose):
    rospy.wait_for_service('iris_sami/pose')
    try:
        poseServ = rospy.ServiceProxy('iris_sami/pose', PoseGoal)
        resp = poseServ(*pose)
        return resp.feedback
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def samiGripService():
    rospy.wait_for_service('iris_sami/grip')
    try:
        grip_serv = rospy.ServiceProxy('iris_sami/grip', NoArguments)
        resp = grip_serv()
        return resp.feedback
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def samiReleaseService():
    rospy.wait_for_service('iris_sami/release')
    try:
        release_serv = rospy.ServiceProxy('iris_sami/release', NoArguments)
        resp = release_serv()
        return resp.feedback
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)