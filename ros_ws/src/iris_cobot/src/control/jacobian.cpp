#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <iris_cobot/JointSpeed.h>


geometry_msgs::WrenchStamped *wrench_velocity_ptr;

void wrenchVelSub(geometry_msgs::WrenchStamped wrench_vel_msg)
{
    *wrench_velocity_ptr = wrench_vel_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jacobian");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    // Moveit controller and Kinematic state  
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    
    robot_model::RobotModelConstPtr kinematic_model = move_group.getRobotModel();
    robot_state::RobotStatePtr kinematic_state = move_group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    geometry_msgs::WrenchStamped wrench_velocity;
    wrench_velocity_ptr = &wrench_velocity;
    ros::Subscriber lin_vel_sub = nh.subscribe("wrench_velocity", 1, wrenchVelSub);

    ros::Publisher joint_speed_pub = nh.advertise<iris_cobot::JointSpeed>("joint_speeds", 1);

    ROS_INFO("Jacobian node listening to linear/angular and publishing to joint_speeds");

    ros::Rate rate(500);
    while(ros::ok())
    {
        // Get translation and rotation from linear and angular velocity messages
        Eigen::Vector3d translation;
        Eigen::Vector3d rotation;

        geometry_msgs::Vector3 linear = wrench_velocity_ptr->wrench.force;
        geometry_msgs::Vector3 angular = wrench_velocity_ptr->wrench.torque;

        translation << linear.x , linear.y , linear.z;
        rotation << angular.x , angular.y, angular.z;

        // Create Kinematic State
        kinematic_state->setJointGroupPositions(joint_model_group, move_group.getCurrentJointValues());

        // Jacobian Matrix
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        kinematic_state->getJacobian(joint_model_group,
                                    kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                    reference_point_position, jacobian);

        // Velocity builder
        Eigen::VectorXd velocity(6);
        velocity << translation, rotation;

        // Jacobian Pseudo Inverse
        Eigen::MatrixXd jacobian_inv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        
        // Joint Velocities
        Eigen::VectorXd joint_velocities = jacobian_inv * velocity;

        iris_cobot::JointSpeed joint_speed_msg;
        for (int i = 0; i < joint_velocities.size(); i++) 
        {
            joint_speed_msg.joint_speeds.push_back(joint_velocities(i));
        }

        joint_speed_pub.publish(joint_speed_msg);

        rate.sleep();
    }
    
    ros::shutdown();

    return 0;
}