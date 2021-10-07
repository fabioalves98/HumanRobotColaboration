#include <iostream>
#include <numeric>
#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <iris_cobot/PFVector.h>

template<typename T>
using sVec = std::vector<T>;

ros::Publisher *attraction_pub_ptr;
ros::Publisher *cur_marker_pub_ptr;
ros::Publisher *goal_marker_pub_ptr;

sVec<sVec<double>> *trajectory_ptr;
ros::Publisher *traj_marker_pub_ptr;
visualization_msgs::MarkerArray *traj_marker_ptr;

robot_state::RobotStatePtr kinematic_state;
const robot_state::JointModelGroup* joint_model_group;
moveit::planning_interface::MoveGroupInterface *ur10e_mg_ptr;

int previous_idx = 0;


visualization_msgs::Marker sphereMarker(geometry_msgs::TransformStamped marker_tf, 
    sVec<double> rgbas, int id = 0)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = marker_tf.transform.translation.x;
    marker.pose.position.y = marker_tf.transform.translation.y;
    marker.pose.position.z = marker_tf.transform.translation.z;
    marker.pose.orientation = marker_tf.transform.rotation;
    marker.color.r = rgbas[0];
    marker.color.g = rgbas[1];
    marker.color.b = rgbas[2];
    marker.color.a = rgbas[3];
    marker.scale.x = rgbas[4];
    marker.scale.y = rgbas[4];
    marker.scale.z = rgbas[4];
    return marker;
}

geometry_msgs::TransformStamped forwardKinematics(sVec<double> joint_values)
{
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("flange");
    return tf2::eigenToTransform(end_effector_state);
}

int findGoalIdx()
{
    sVec<double> cur_joints = ur10e_mg_ptr->getCurrentJointValues();
    sVec<double> differences;

    for (auto point : *trajectory_ptr)
    {
        sVec<double> diff;
        for (int i = 0; i < 6; i++)
        {
            diff.push_back(abs(point[i] - cur_joints[i]));
        }
        differences.push_back(std::accumulate(diff.begin(), diff.end(), 0.0));
    }

    // std::cout << "Diferences" << std::endl;
    // for (auto elem : differences)
    // {
    //     std::cout << elem << ' ';
    // }
    // std::cout << std::endl;

    int min_dist_idx = std::distance(std::begin(differences), 
        std::min_element(std::begin(differences), std::end(differences)));

    // std::cout << "Min Dist Idx = " << min_dist_idx << std::endl;
    
    // If it reaches end of trajectory, revert
    if (min_dist_idx == trajectory_ptr->size() - 1)
    {
        // std::cout << "REVERSING" << std::endl;
        std::reverse(trajectory_ptr->begin(), trajectory_ptr->end());
        min_dist_idx = 0;
        previous_idx = 0;
    }

    // Make sure robot keeps the same sirection when following trajectory
    if (min_dist_idx < previous_idx)
    {
        min_dist_idx = previous_idx;
    }
    previous_idx = min_dist_idx;

    return min_dist_idx + 1;
}


void attraction(const ros::TimerEvent& event)
{
    int current_idx = findGoalIdx();
    sVec<double> cur_joint_values = trajectory_ptr->at(current_idx);
    sVec<double> goal_joint_values;
    if (current_idx != trajectory_ptr->size() - 1)
    {
        // goal_joint_values = trajectory_ptr->back();
        goal_joint_values = trajectory_ptr->at(current_idx + 1);
    }
    else
    {
        goal_joint_values = cur_joint_values;
    }

    // std::cout << current_idx << std::endl;
    // for (auto joint : cur_joint_values)
    // {
    //     std::cout << joint << ' ';
    // }
    // std::cout << std::endl;

    // Current pose
    geometry_msgs::PoseStamped ee_pose = ur10e_mg_ptr->getCurrentPose();
    // std::cout << ee_pose << std::endl;

    tf2::Transform ee_tf;
    ee_tf.setOrigin(tf2::Vector3(ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z));
    tf2::Quaternion ee_rot;
    tf2::convert(ee_pose.pose.orientation, ee_rot);
    ee_tf.setRotation(ee_rot);

    // Current Point Pose
    tf2::Stamped<tf2::Transform> cur_tf;
    geometry_msgs::TransformStamped cur_tf_msg = forwardKinematics(cur_joint_values);
    tf2::fromMsg(cur_tf_msg, cur_tf);

    // Publish Current Point TF
    visualization_msgs::Marker cur_marker = sphereMarker(cur_tf_msg, {1, 1, 0, 1, 0.1}, 0);
    cur_marker_pub_ptr->publish(cur_marker);
    traj_marker_pub_ptr->publish(*traj_marker_ptr);

    // Goal Point Pose
    tf2::Stamped<tf2::Transform> goal_tf;
    geometry_msgs::TransformStamped goal_tf_msg = forwardKinematics(goal_joint_values);
    tf2::fromMsg(goal_tf_msg, goal_tf);

    // Publish Goal Point TF
    visualization_msgs::Marker goal_marker = sphereMarker(goal_tf_msg, {0, 1, 0, 1, 0.1}, 0);
    goal_marker_pub_ptr->publish(goal_marker);
    traj_marker_pub_ptr->publish(*traj_marker_ptr);

    // Difference from goal pose to target pose
    tf2::Transform diff_tf;
    diff_tf = cur_tf * ee_tf.inverse();

    // Obtain linear velocity using current and goal points position
    Eigen::Vector3d lin_vel;
    lin_vel << (goal_tf.getOrigin().getX()+cur_tf.getOrigin().getX())/2 - ee_tf.getOrigin().getX(),
               (goal_tf.getOrigin().getY()+cur_tf.getOrigin().getY())/2 - ee_tf.getOrigin().getY(),
               (goal_tf.getOrigin().getZ()+cur_tf.getOrigin().getZ())/2 - ee_tf.getOrigin().getZ();

    if (lin_vel.norm() < 0.01)
    {
        lin_vel << 0, 0, 0;
    }
    else
    {
        lin_vel = lin_vel.normalized();
    }

    geometry_msgs::Vector3 lin_vel_msg;
    tf2::toMsg(lin_vel, lin_vel_msg);
    
    geometry_msgs::Vector3 ang_vel_msg;
    double roll, pitch, yaw;
    tf2::Matrix3x3(diff_tf.getRotation()).getRPY(roll, pitch, yaw);

    if (abs(roll) > 0.05 || abs(pitch) > 0.05 || abs(yaw) > 0.05)
    {
        ang_vel_msg.x = roll;
        ang_vel_msg.y = pitch;
        ang_vel_msg.z = yaw;
    }
    else
    {
        ang_vel_msg.x = 0;
        ang_vel_msg.y = 0;
        ang_vel_msg.z = 0;
    }

    iris_cobot::PFVector attraction;
    attraction.linear_velocity = lin_vel_msg;
    attraction.angular_velocity = ang_vel_msg;

    attraction_pub_ptr->publish(attraction);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "attraction");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    // Moveit controller and Kinematic state 
    moveit::planning_interface::MoveGroupInterface ur10e_mg("manipulator");
    ur10e_mg_ptr = &ur10e_mg;

    kinematic_state = ur10e_mg.getCurrentState();
    robot_model::RobotModelConstPtr kinematic_model = ur10e_mg.getRobotModel();
    joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    // Cur Marker Publisher
    ros::Publisher cur_marker_pub = nh.advertise<visualization_msgs::Marker>("cur", 1);
    cur_marker_pub_ptr = &cur_marker_pub;

    // Goal Marker Publisher
    ros::Publisher goal_marker_pub = nh.advertise<visualization_msgs::Marker>("goal", 1);
    goal_marker_pub_ptr = &goal_marker_pub;

    // Trajectory Marker Publisher
    ros::Publisher trajectory_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 1);
    traj_marker_pub_ptr = &trajectory_marker_pub;

    // Obtain Robot current state
    robot_state::RobotStatePtr start_state = ur10e_mg.getCurrentState();


    // Start Position
    sVec<double> start_joints = {
        1.8872361183166504, -2.362852712670797, -1.6420011520385742, 
        -0.6762150090983887, 1.5861048698425293, -0.45677310625185186
    };
    // Waypoint 1
    sVec<double> waypoint_1 = {
        1.7148256301879883, -1.995969911614889, -1.9703092575073242, 
        0.08445851385083003, 2.0642237663269043, -0.4083760420428675
    };
    // Waipoint 2
    sVec<double> waypoint_2 = {
        3.1301183700561523, -1.707872053185934, -2.3773908615112305, 
        0.13671223699536128, 0.9651718139648438, 0.5363349914550781
    };
    // End Position
    sVec<double> end_joints = 
    {
        2.8675928115844727, -2.211241384545797, -1.9777193069458008, 
        -0.4927595418742676, 1.553483009338379, 0.5230627059936523
    };

    // Global Trajectory Creator
    sVec<sVec<double>> trajectory;
    trajectory_ptr = &trajectory;

    visualization_msgs::MarkerArray trajectory_markers;
    traj_marker_ptr = &trajectory_markers;
    int marker_id = 0;

    sVec<sVec<double>> waypoints = {start_joints, waypoint_1, waypoint_2, end_joints};

    for (int i = 0; i < waypoints.size() - 1; i++)
    {
        // Set Start State
        start_state->setJointGroupPositions(joint_model_group, waypoints[i]);

        // Trajectory Creator
        ur10e_mg.setStartState(*start_state);
        ur10e_mg.setJointValueTarget(waypoints[i + 1]);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        ur10e_mg.plan(my_plan);

        std::cout << "Trajectory with " << my_plan.trajectory_.joint_trajectory.points.size() 
                << "points" << std::endl;
        
        for (auto point : my_plan.trajectory_.joint_trajectory.points)
        {   
            trajectory.push_back(point.positions);
            trajectory_markers.markers.push_back(sphereMarker(forwardKinematics(point.positions),
                {1, 0, 0, 0.5, 0.08} , marker_id));
            marker_id++;
        }
    }

    // Check for overlaps in trajectory points and remove them
    sVec<sVec<double>> duplicate_joints;
    for (int i = 0; i < trajectory.size() - 1; i ++)
    {
        sVec<double> diff;
        for (int j = 0; j < 6; j++)
        {
            diff.push_back(abs(trajectory[i][j] - trajectory[i + 1][j]));
        }
        double diff_value =  std::accumulate(diff.begin(), diff.end(), 0.0);
        if (diff_value < 0.1)
        {
            duplicate_joints.push_back(trajectory[i]);
        }
    }
    for (sVec<double> point : duplicate_joints)
    {
        auto it = std::find(trajectory.begin(),trajectory.end(), point);
        if (it != trajectory.end()) 
        {
            trajectory.erase(it);
        }
    }

    // Print Trajectory
    for (auto point : trajectory)
    {
        for (auto joint : point)
        {
            std::cout << joint << ' ';
        }
        std::cout << std::endl;
    }
    std::cout << "Global Trajectory with " << trajectory.size() << " points" << std::endl;

    // Attraction Vector Publisher
    ros::Publisher attraction_pub = nh.advertise<iris_cobot::PFVector>("attraction", 1);
    attraction_pub_ptr = &attraction_pub;

    // Goal trajectory point subscriber
    // ros::Subscriber sub = nh.subscribe("trajectory_point", 1, attraction);
    ros::Timer timer = nh.createTimer(ros::Duration(1/500), attraction);

    ROS_INFO("Atraction node listening to trajectory_point and publishing to attraction");
    
    ros::waitForShutdown();

    return 0;
}