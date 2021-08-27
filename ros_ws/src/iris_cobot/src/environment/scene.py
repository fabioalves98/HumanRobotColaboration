#!/usr/bin/env python
import rospy
from math import pi
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
from moveit_commander.planning_scene_interface import PlanningSceneInterface


scene = None

def wait_for_state_update(scene_arg, object_name="", object_is_known=False, object_is_attached=False, timeout=4):
    
    start = rospy.get_time()
    seconds = rospy.get_time()
    while(seconds - start < timeout) and not rospy.is_shutdown():
        # Check if object is in attached objects
        attached_objects = scene_arg.get_attached_objects([object_name])
        is_attached = len(attached_objects.keys()) > 0
        # Check if object is in world objects
        is_known = object_name in scene_arg.get_known_object_names()
        if (object_is_attached == is_attached) and (object_is_known == is_known):
            return True
        
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    return False


def delete_objects():
    scene.remove_world_object()
    wait_for_state_update(scene)


def main():
    global scene

    rospy.init_node('setup_scene', anonymous=False)
    rospy.on_shutdown(delete_objects)
    
    scene = PlanningSceneInterface(synchronous=True)
    # camera_transform = getTransform('base_link', 'camera_link')
    
    # # Camera Box Scene
    # pos = objectToArray(camera_transform.transform.translation)
    # ori = objectToArray(camera_transform.transform.rotation)
    # camera_box = newPoseStamped(pos, ori, "base_link")
    # scene.add_box("camera_box", camera_box, size=(CAMERA_WIDTH, CAMERA_LENGTH, CAMERA_HEIGHT))
    # status = wait_for_state_update(scene, "camera_box", object_is_known=True)
    # rospy.loginfo("Created camera box") if status else rospy.logwarn("Failed creating camera box")

    # Robot Desk Scene
    base_plane = PoseStamped()
    base_plane.header.frame_id = "base_link"
    base_plane.pose.position = Point(*[0, 0, -0.05])
    base_plane.pose.orientation = Quaternion(*quaternion_from_euler(pi/2, 0, pi/4))
    scene.add_box("base_plane", base_plane, size=(1, 0.05, 1))
    status = wait_for_state_update(scene, "base_plane", object_is_known=True) 
    rospy.loginfo("Created base plane") if status else rospy.logwarn("Failed creating base plane")

    rospy.spin()


if __name__ == "__main__":
    main()