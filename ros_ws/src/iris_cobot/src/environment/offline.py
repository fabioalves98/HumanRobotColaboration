#!/usr/bin/env python
import rospy, sys
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory
import moveit_commander

from iris_cobot.msg import JointSpeed

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

        # Finish trajectory
        if self.cur_point == len(self.points) - 1:
            self.finished = True
            return False
        
        return self.points[self.cur_point]


    def get_speed(self):
        if self.has_past_cur_point:
            speed = [n - c for n, c in zip (self.points[self.cur_point + 1].positions, 
                                            self.points[self.cur_point].positions)]
        else:
            speed = [n - c for n, c in zip (self.points[self.cur_point].positions, 
                                            self.points[self.cur_point - 1].positions)]

        return speed
    

    def is_finished(self):
        return self.finished


def list_2_str(list):
    return str([round(elem, 3) for elem in list])


def main():
    rospy.init_node('offline', anonymous=False)
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    move_group = moveit_commander.MoveGroupCommander("manipulator")

    # RViz trajectory viewer
    display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=1)

    # print(move_group.get_planning_time())
    # print("Available Planners")
    # for planner in move_group.get_interface_description().planner_ids:
    #     print(planner)   
    
    # RRTConnect - Faster
    # RRTStar - Optimal version of RRT, Slower
    # print(move_group.set_planner_id('manipulator[RRTstar]'))
    # print(move_group.get_planner_id())

    # Define a pose wich is 20cm lower than the current pose
    pose_goal = PoseStamped()
    pose_goal.pose.position.x = 0.5
    pose_goal.pose.position.y = 0.5
    pose_goal.pose.position.z = 0.5

    move_group.set_start_state(robot.get_current_state())
    plan = move_group.plan(pose_goal)

    points = plan.joint_trajectory.points
    print('Trajecotry with %d points' % len(points))

    # Instantiate Trajectory Executioner
    traj_exec = TrajectoryExecutioner(points)

    # Joint Speed Control
    traj_point_pub = rospy.Publisher('/trajectory_point', JointTrajectoryPoint, queue_size=1)

    # Speed Control Cycle
    rate = rospy.Rate(500)
    while not traj_exec.is_finished():
        # Get current pose 
        current_q = move_group.get_current_joint_values()

        # Obtain next instruction from trajectory executioner
        current_point = traj_exec.localize(current_q)
        # print(current_point)
        # print(type(current_point))
        if current_point:
            joint_speeds = traj_exec.get_speed()
            # print(list_2_str(joint_speeds))
            traj_point_pub.publish(current_point)
        
        rate.sleep()

        
if __name__ == "__main__":
    main()