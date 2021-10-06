#!/usr/bin/env python
import rospy, sys
from math import radians
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
import moveit_commander
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot import RobotCommander
from tf.transformations import quaternion_from_euler

ur10e_com = None # type: RobotCommander
ur10e_mg = None # type: MoveGroupCommander
display_trajectory_pub = None # type: rospy.Publisher


def plan(event=None):
    # RRTConnect - Faster
    # RRTStar - Optimal version of RRT, Slower
    # ur10e_mg.set_planner_id('manipulator[RRTstar]')
    print(ur10e_mg.get_planner_id())

    # Define a start pose 
    robot_state = ur10e_com.get_current_state()
    start_joint_state = [-2.9385049958219014, -2.272357028717571, -1.312998376075341, 
                         -0.4159580635132407, 1.9500205901742653, -0.40501343460914807, 0, 0]
    robot_state.joint_state.position = start_joint_state

    # print("\nInitial Robot State")
    # print(robot_state)

    # Define a target pose
    pose_goal = PoseStamped()
    pose_goal.pose.position = Point(1, 0, 0.3)
    pose_goal.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(0)))

    # Define a target joint state
    joint_goal = JointState()
    joint_goal.name = robot_state.joint_state.name
    joint_goal.position = [-1.4639981907416555, -2.305494986407135, -1.221936453462713, 
                           -0.6399023596943474, 0.9668708874523881, 0.7535286047726073, 0, 0]

    
    # Create plan between the 2 poses
    ur10e_mg.set_start_state(robot_state)

    # print("\nRobot State")
    # print(robot_state)

    plan = ur10e_mg.plan(joint_goal)

    # print("\nPlan")
    # for point in plan.joint_trajectory.points:
    #     print(point.positions)

    # Display Trajectory in RViz
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot_state
    display_trajectory.trajectory.append(plan)
    display_trajectory_pub.publish(display_trajectory)

    points = plan.joint_trajectory.points
    print('Trajecotry with %d points' % len(points))


def main():
    rospy.init_node('planner', anonymous=False)
    
    moveit_commander.roscpp_initialize(sys.argv)

    global ur10e_com, ur10e_mg, display_trajectory_pub
    ur10e_com = RobotCommander()
    ur10e_mg = MoveGroupCommander("manipulator")   

    rospy.set_param('/move_group/ompl/maximum_waypoint_distance', 0.2)
    
    display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=1)

    # Available Planners
    for planner in ur10e_mg.get_interface_description().planner_ids:
        print(planner) 

    # rospy.Timer(rospy.Duration(1), plan)
    plan()

    rospy.spin()

        
if __name__ == "__main__":
    main()