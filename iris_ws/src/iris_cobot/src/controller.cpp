#include <iostream>
#include <ros/ros.h>
#include <chrono>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    // Cucrrent Joint Values    
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();

    std::cout << "Current Joint Values - " << current_joint_values.size() << std::endl;
    for (auto i : current_joint_values)
        std::cout << i << '\n';
    std::cout << std::endl;
    

    // Current EE pose
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    const robot_state::JointModelGroup* joint_model_group = 
        move_group.getCurrentState()->getJointModelGroup("manipulator");
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        std::cout << "Joint " << joint_names[i].c_str() << " - " << joint_values[i] << std::endl;
    }
    // Pose obtained with robot state - FK
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("gripper_link");
    Eigen::Vector3d position = end_effector_state.translation();
    std::cout << "Position\n" << position << std::endl;
    Eigen::Quaterniond orientation(end_effector_state.rotation());
    std::cout << "Orientation\n" << orientation.vec() << '\n' << orientation.w() << std::endl;
    // Pose obtained directly from move group
    std::cout << move_group.getCurrentPose() << std::endl;


    // Goal EE pose
    Eigen::Vector3d new_position = position;
    new_position[0] += 0.1;
    new_position[1] += 0.1;
    new_position[2] += 0.1;
    Eigen::Quaterniond new_orientation = orientation;

    while(true)
    {   
        // auto start = std::chrono::high_resolution_clock::now();
        // Get current position
        current_joint_values = move_group.getCurrentJointValues();
        kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
        const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("gripper_link");
        position = end_effector_state.translation();

        // Jacobian Matrix
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        kinematic_state->getJacobian(joint_model_group,
                                    kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                    reference_point_position, jacobian);
        // std::cout << "Jacobian: \n" << jacobian << "\n" << std::endl;

        // Movement calculator
        Eigen::VectorXd displacement(6);
        displacement << new_position - position, 0, 0, 0;
        // std::cout << "Displacement" << std::endl << displacement << std::endl;
        double scaling_factor = 0.1;
        displacement *= scaling_factor;
        // Jacobian Pseudo Inverse
        Eigen::MatrixXd jacobian_inv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        // New joint increments
        Eigen::VectorXd joint_increments = jacobian_inv * displacement;
        // std::cout << "Joint Increments" << std::endl << joint_increments << std::endl;
        Eigen::VectorXd new_joints = Eigen::VectorXd::Map(current_joint_values.data(), 6) + joint_increments;
        // std::cout << "New Joints" << std::endl << new_joints << std::endl;

        std::vector<double> joint_command;
        for (int i = 0; i < new_joints.size(); i++) 
        {
            joint_command.push_back(new_joints(i));
        }

        // auto finish = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> elapsed = finish - start;
        // std::cout << "Elapsed time: " << elapsed.count() << " s\n";
        // std::cout << "Frequency: " << 1 / elapsed.count() << " hz\n";

        move_group.setJointValueTarget(joint_command);
        
        // auto start = std::chrono::high_resolution_clock::now();
        // move_group.move();
        // auto finish = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> elapsed = finish - start;
        // std::cout << "Elapsed time: " << elapsed.count() << " s\n";
        // std::cout << "Frequency: " << 1 / elapsed.count() << " hz\n";
        
        move_group.stop();
        move_group.asyncMove();
        ros::Duration(0.05).sleep();
    }
    
    ros::shutdown();

    return 0;
}