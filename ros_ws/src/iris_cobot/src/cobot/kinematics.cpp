#include <iostream>
#include <ros/ros.h>
#include <chrono>

#include <tf2_eigen/tf2_eigen.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <urdf_model/model.h>
#include <urdf/urdfdom_compatibility.h>

#include <iris_cobot/ForwardKinematics.h>

robot_state::RobotStatePtr kinematic_state;
const robot_state::JointModelGroup* joint_model_group;

bool forwardKinematics(iris_cobot::ForwardKinematics::Request& req, iris_cobot::ForwardKinematics::Response& res)
{
    kinematic_state->setJointGroupPositions(joint_model_group, req.joint_state.position);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("flange");
    res.pose = tf2::eigenToTransform(end_effector_state).transform;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics");
    ros::NodeHandle n;

    // Information about robot and joints
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    
    // Global variable setting
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    ros::ServiceServer fk_serv = n.advertiseService("forward_kinematics", forwardKinematics);


    // Forward Kinematics
    // kinematic_state->setToRandomPositions(joint_model_group);
    // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    // for (std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    // }

    // const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("gripper_link");

    // ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    // ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    // // Inverse Kinematics
    // double timeout = 0.1;
    // bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    // if (found_ik)
    // {
    //     kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    //     for (std::size_t i = 0; i < joint_names.size(); ++i)
    //     {
    //         ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    //     }
    // }
    // else
    // {
    //     ROS_INFO("Did not find IK solution");
    // }

    // std::cout << joint_model_group->getLinkModelNames().back() << std::endl;

    // // Jacobian
    // Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    // Eigen::MatrixXd jacobian;
    // kinematic_state->getJacobian(joint_model_group,
    //                         kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
    //                         reference_point_position, jacobian);
    // ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

    
    ros::spin();

    return 0;
}