#!/usr/bin/env python
import rospy, sys
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf.transformations import quaternion_from_euler
from math import pow, sqrt
import moveit_commander

from iris_cobot.msg import Obstacles

ur10e_mg = None
camera_tf = None

def repulsion(obstacles):
    ee_center = ur10e_mg.get_current_pose().pose.position
    print('EE - %f, %f, %f' % (ee_center.x, ee_center.y, ee_center.z))

    for obs_idx in range(obstacles.size):
        obs_point = PointStamped(point=obstacles.centers[obs_idx])
        obs_center = do_transform_point(obs_point, camera_tf).point
        obs_radius = obstacles.radiuses[obs_idx]
        distance = sqrt(pow(ee_center.x - obs_center.x, 2) + pow(ee_center.y - obs_center.y, 2) + 
                        pow(ee_center.z - obs_center.z, 2)) - obs_radius
        print('%d - %f' % (obs_idx, distance))

    print('')
    

def main():
    rospy.init_node('repulsion', anonymous=False)
    
    moveit_commander.roscpp_initialize(sys.argv)
    global ur10e_mg
    ur10e_mg = moveit_commander.MoveGroupCommander("manipulator")

    # Obtain camera tf
    global camera_tf
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    while True:
        try:
            camera_tf = tfBuffer.lookup_transform('world', 'camera_depth_optical_frame', rospy.Time())
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    rospy.Subscriber('obstacles', Obstacles, repulsion)

    rospy.spin()


if __name__ == "__main__":
    main()