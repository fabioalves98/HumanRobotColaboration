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


robot_state::RobotStatePtr kinematic_state;
const robot_state::JointModelGroup* joint_model_group;
moveit::planning_interface::MoveGroupInterface *move_group_ptr;


void attraction(trajectory_msgs::JointTrajectoryPoint point)
{
    std::vector<double> joint_values = point.positions;

    // Current pose
    geometry_msgs::PoseStamped ee_pose = move_group_ptr->getCurrentPose();
    std::cout << ee_pose << std::endl;

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

    std::cout << diff_tf.getOrigin().getX() << ", "
              << diff_tf.getOrigin().getY() << ", "
              << diff_tf.getOrigin().getZ() << std::endl;

    std::cout << diff_tf.getRotation().getX() << ", "
              << diff_tf.getRotation().getY() << ", "
              << diff_tf.getRotation().getZ() << ", "
              << diff_tf.getRotation().getW() << std::endl;
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


    // Goal trajectory point subscriber
    ros::Subscriber sub = nh.subscribe("trajectory_point", 1, attraction);
    
    ros::waitForShutdown();

    return 0;
}