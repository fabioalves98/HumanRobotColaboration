#include <iostream>
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

ros::Publisher *attraction_pub_ptr;

robot_state::RobotStatePtr kinematic_state;
const robot_state::JointModelGroup* joint_model_group;
moveit::planning_interface::MoveGroupInterface *move_group_ptr;


void attraction(trajectory_msgs::JointTrajectoryPoint point)
{
    std::vector<double> joint_values = point.positions;

    // Current pose
    geometry_msgs::PoseStamped ee_pose = move_group_ptr->getCurrentPose();
    // std::cout << ee_pose << std::endl;

    tf2::Transform ee_tf;
    ee_tf.setOrigin(tf2::Vector3(ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z));
    tf2::Quaternion ee_rot;
    tf2::convert(ee_pose.pose.orientation, ee_rot);
    ee_tf.setRotation(ee_rot);

    // Target Pose
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("flange");
    tf2::Stamped<tf2::Transform> goal_tf;
    geometry_msgs::TransformStamped goal_tf_msg = tf2::eigenToTransform(end_effector_state);
    tf2::fromMsg(goal_tf_msg, goal_tf);

    // Difference from goal pose to target pose
    tf2::Transform diff_tf;
    diff_tf = goal_tf * ee_tf.inverse();

    Eigen::Vector3d lin_vel;
    lin_vel << goal_tf.getOrigin().getX() - ee_tf.getOrigin().getX(),
               goal_tf.getOrigin().getY() - ee_tf.getOrigin().getY(),
               goal_tf.getOrigin().getZ() - ee_tf.getOrigin().getZ();

    if (lin_vel.norm() < 0.01)
    {
        lin_vel << 0, 0, 0;
    }
    else if (lin_vel.norm() > 0.1)
    {
        lin_vel = lin_vel.normalized();
    }

    geometry_msgs::Vector3 lin_vel_msg;
    tf2::toMsg(lin_vel, lin_vel_msg);
    
    geometry_msgs::Vector3 ang_vel_msg;
    double roll, pitch, yaw;
    tf2::Matrix3x3(diff_tf.getRotation()).getRPY(roll, pitch, yaw);

    if (roll > 0.05 || pitch > 0.05 || yaw > 0.05)
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
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group_ptr = &move_group;

    kinematic_state = move_group.getCurrentState();
    robot_model::RobotModelConstPtr kinematic_model = move_group.getRobotModel();
    joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    // Attraction Vector Publisher
    ros::Publisher attraction_pub = nh.advertise<iris_cobot::PFVector>("attraction", 1);
    attraction_pub_ptr = &attraction_pub;

    // Goal trajectory point subscriber
    ros::Subscriber sub = nh.subscribe("trajectory_point", 1, attraction);

    ROS_INFO("Atraction node listening to trajectory_point and publishing to attraction");
    
    ros::waitForShutdown();

    return 0;
}