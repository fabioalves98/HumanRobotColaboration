#!/usr/bin/env python
import rospy, sys
from math import radians
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory
import moveit_commander
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot import RobotCommander
from tf.transformations import quaternion_from_euler


GAZEBO_LEFT = [-2.9385049958219014, -2.272357028717571, -1.312998376075341, 
               -0.4159580635132407, 1.9500205901742653, -0.40501343460914807, 0, 0]
GAZEBO_RIGHT = [-1.4639981907416555, -2.305494986407135, -1.221936453462713, 
                -0.6399023596943474, 0.9668708874523881, 0.7535286047726073, 0, 0]

REAL_LEFT = [1.7148256301879883, -1.995969911614889, -1.9703092575073242, 
             0.08445851385083003, 2.0642237663269043, -0.4083760420428675, 0, 0]
REAL_RIGHT = [3.1301183700561523, -1.707872053185934, -2.3773908615112305, 
             0.13671223699536128, 0.9651718139648438, 0.5363349914550781, 0, 0]


# TODO: Entire class needs restructure and better way to check current point and finish trajectory
class TrajectoryExecutioner:
    def __init__(self, points):
        self.points = points
        self.cur_point = 0
        self.prev_point = 0
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
            self.cur_point = 0
            self.prev_point = 0
            return self.points[0]
        
        print(self.prev_point, self.cur_point)
        # Make sure robot doesn't go back on trajectory because collision
        if self.cur_point < self.prev_point:
            self.cur_point = self.prev_point

        self.prev_point = self.cur_point

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
    # move_group.set_planner_id('manipulator[RRTstar]')
    print(move_group.get_planner_id())

    # Define a start pose 
    robot_state = robot.get_current_state()
    start_joint_state = REAL_LEFT
    robot_state.joint_state.position = start_joint_state

    # Define a target pose
    pose_goal = PoseStamped()
    pose_goal.pose.position = Point(0.5, 0.5, 0.2)
    pose_goal.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(45)))

    # Define a target joint state
    joint_goal = JointState()
    joint_goal.name = robot_state.joint_state.name
    joint_goal.position = REAL_RIGHT


    # Create plan between the 2 poses
    move_group.set_start_state(robot_state)
    plan = move_group.plan(joint_goal)

    # Display Trajectory in RViz
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot_state
    display_trajectory.trajectory.append(plan)
    display_trajectory_pub.publish(display_trajectory)

    points = plan.joint_trajectory.points
    print('Trajecotry with %d points' % len(points))

    # Create a single trajecotry point for debung
    traj_point = JointTrajectoryPoint(positions=[-3.56459, -1.45647, -2.34757, 0.05356, 1.26850, 0.20732])

    # Instantiate Trajectory Executioner
    traj_exec = TrajectoryExecutioner(points)

    # Joint Speed Control
    traj_point_pub = rospy.Publisher('/trajectory_point', JointTrajectoryPoint, queue_size=1)

    # Speed Control Cycle
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        # # Get current pose 
        current_q = move_group.get_current_joint_values()
        
        # Obtain next instruction from trajectory executioner
        current_point = traj_exec.localize(current_q)

        if current_point:
            traj_point_pub.publish(current_point)

        # traj_point_pub.publish(traj_point)
        
        rate.sleep()

        
if __name__ == "__main__":
    main()