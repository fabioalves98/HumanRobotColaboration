#!/usr/bin/env python
import numpy as np
import rospy, sys
from geometry_msgs.msg import PointStamped, Vector3
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from math import pow, sqrt
import moveit_commander
from moveit_commander.move_group import MoveGroupCommander

import cobot.helpers as helpers
from iris_cobot.msg import Obstacles, PFVector


ur10e_mg = None # type: MoveGroupCommander
camera_tf = None
repulsion_pub = None # type: rospy.Publisher

MAX_DIST = 0.4


def repulsion(obstacles_msg, real):
    ee_center = ur10e_mg.get_current_pose().pose.position

    # Get distances from obstacles to EEF
    obstacles = np.empty([0, 5])
    for obs_idx in range(obstacles_msg.size):
        # Check wheter coordinates com from camera or direct obstacle coordinates
        if real:
            # Transform obstacle coordinates from camera frame to world
            obs_point = PointStamped(point=obstacles_msg.centers[obs_idx])
            obs_center = do_transform_point(obs_point, camera_tf).point
        else:
            # Subtract 10cm because gazebo simulated world differs from robot position in 10cm
            obs_center = obstacles_msg.centers[obs_idx]
            obs_center.z -= 0.1

        obs_radius = obstacles_msg.radiuses[obs_idx]

        distance = sqrt(pow(ee_center.x - obs_center.x, 2) + pow(ee_center.y - obs_center.y, 2) + 
                        pow(ee_center.z - obs_center.z, 2))
        if distance < MAX_DIST:
            obstacle = helpers.pointToList(obs_center)
            obstacle.append(obs_radius)
            obstacle.append(distance)
            obstacles = np.append(obstacles, [obstacle], axis=0)

    # Get obstacle with minimum distance and calculate repulsion vector
    repulsion_msg = PFVector()

    if obstacles.shape != (0,5):
        min_obs_idx = np.argmin(obstacles, axis=0)[4]
        ee_center_arr = np.array(helpers.pointToList(ee_center))
        obs_center_arr = obstacles[min_obs_idx][:3]
        rep_vector = np.subtract(ee_center_arr, obs_center_arr)
        rep_vector = rep_vector/np.linalg.norm(rep_vector)
        
        distance = obstacles[min_obs_idx][4]
        rep_factor = (MAX_DIST - (distance - 0.1)) * 1 / (MAX_DIST + 0.1)
        # print(distance, rep_factor)
        rep_vector = rep_vector * rep_factor

        # print(min_obs_idx, obstacles[min_obs_idx], rep_vector)
        repulsion_msg.linear_velocity = Vector3(*rep_vector)
    else:
        repulsion_msg.linear_velocity = Vector3(0, 0, 0)

    if (ee_center.z > 0.2):
        # print('ATIVATED')
        repulsion_pub.publish(repulsion_msg)
    else:
        # print('DEACTIVATED')
        repulsion_pub.publish(PFVector(Vector3(0, 0 ,0), Vector3(0, 0, 0)))


def main():
    rospy.init_node('repulsion', anonymous=False)
    
    moveit_commander.roscpp_initialize(sys.argv)
    global ur10e_mg
    ur10e_mg = MoveGroupCommander("manipulator")
    ur10e_mg.set_end_effector_link("gripper_link")

    # Obtain camera tf
    global camera_tf
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    while True:
        try:
            camera_tf = tfBuffer.lookup_transform('base_link', 'camera_depth_optical_frame', rospy.Time())
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
    
    global repulsion_pub
    repulsion_pub = rospy.Publisher('repulsion', PFVector, queue_size=1)

    rospy.Subscriber('obstacles', Obstacles, repulsion, True)
    # rospy.Subscriber('obstacles_fake', Obstacles, repulsion, False)

    rospy.loginfo('Repulsion node listening to obstacles and publishing to repulsion')

    rospy.spin()


if __name__ == "__main__":
    main()