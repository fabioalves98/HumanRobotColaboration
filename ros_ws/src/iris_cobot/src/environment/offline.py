#!/usr/bin/env python
import rospy, sys
from math import radians
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, RobotState
import moveit_commander
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot import RobotCommander
from tf.transformations import quaternion_from_euler


# TODO: Entire class needs restructure and better way to check current point and finish trajectory
class TrajectoryExecutioner:
    def __init__(self, points):
        self.points = points
        self.cur_point = 0
        self.has_past_cur_point = False
        self.finished = False
    

    def localize(self, current_q):
        # Given the current position, define the closest point in the trajectory to start
        distances = [sum([abs(c - p) for c, p in zip(current_q, point.positions)]) for point in self.points]
        cp_idx = distances.index(min(distances))
        distances[cp_idx] = sys.maxsize
        cp_2_idx = distances.index(min(distances))

        # Setting initial variables
        self.cur_point = cp_idx
        self.has_past_cur_point = cp_2_idx > cp_idx

        # Finish trajectory - Invert for now
        if self.cur_point == len(self.points) - 1:
            self.points.reverse()
            return self.points[0]
        
        return self.points[self.cur_point + 1]
    

    def is_finished(self):
        return self.finished


def main():
    rospy.init_node('offline', anonymous=False)
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = RobotCommander()

    move_group = MoveGroupCommander("manipulator")

    # RViz trajectory viewer
    display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=1)

    # print(move_group.get_planning_time())
    # print("Available Planners")
    # for planner in move_group.get_interface_description().planner_ids:
    #     print(planner)   
    
    # RRTConnect - Faster
    # RRTStar - Optimal version of RRT, Slower
    move_group.set_planner_id('manipulator[RRTstar]')
    print(move_group.get_planner_id())

    # Define a target pose
    pose_goal = PoseStamped()
    pose_goal.pose.position.x = 0.5
    pose_goal.pose.position.y = 0.5
    pose_goal.pose.position.z = 0.2
    pose_goal.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(45)))

    # Define a start pose 
    robot_state = robot.get_current_state()
    robot_state.joint_state.position = [0.48711, -1.57165, 1.41912, -2.98844, -1.27704, -0.00034, 0, 0]

    # Create plan between the 2 poses
    move_group.set_start_state(robot_state)
    plan = move_group.plan(pose_goal)

    # Display Trajectory in RViz
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_pub.publish(display_trajectory)

    points = plan.joint_trajectory.points
    print('Trajecotry with %d points' % len(points))

    # Instantiate Trajectory Executioner
    traj_exec = TrajectoryExecutioner(points)

    # Joint Speed Control
    traj_point_pub = rospy.Publisher('/trajectory_point', JointTrajectoryPoint, queue_size=1)

    # Speed Control Cycle
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        # Get current pose 
        current_q = move_group.get_current_joint_values()

        # Obtain next instruction from trajectory executioner
        current_point = traj_exec.localize(current_q)

        if current_point:
            traj_point_pub.publish(current_point)
        
        rate.sleep()

        
if __name__ == "__main__":
    main()