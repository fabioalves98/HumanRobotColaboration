#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <iris_cobot/JointSpeed.h>

bool KEYBOARD = false;

geometry_msgs::Vector3 *linear_velocity_ptr;
geometry_msgs::Vector3 *angular_velocity_ptr;


void linearVelSub(geometry_msgs::Vector3 lin_vel)
{
    linear_velocity_ptr->x = lin_vel.x;
    linear_velocity_ptr->y = lin_vel.y;
    linear_velocity_ptr->z = lin_vel.z;
}

void angularVelSub(geometry_msgs::Vector3 ang_vel)
{
    angular_velocity_ptr->x = ang_vel.x;
    angular_velocity_ptr->y = ang_vel.y;
    angular_velocity_ptr->z = ang_vel.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jacobian");
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
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
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");
    Eigen::Vector3d position = end_effector_state.translation();
    std::cout << "Position\n" << position << std::endl;
    Eigen::Quaterniond orientation(end_effector_state.rotation());
    std::cout << "Orientation\n" << orientation.vec() << '\n' << orientation.w() << std::endl;
    // Pose obtained directly from move group
    std::cout << move_group.getCurrentPose() << std::endl;

    geometry_msgs::Vector3 linear_velocity;
    linear_velocity_ptr = &linear_velocity;
    ros::Subscriber lin_vel_sub = nh.subscribe("linear_velocity", 1, linearVelSub);

    geometry_msgs::Vector3 angular_velocity;
    angular_velocity_ptr = &angular_velocity;
    ros::Subscriber ang_vel_sub = nh.subscribe("angular_velocity", 1, angularVelSub);

    ros::Publisher joint_speed_pub = nh.advertise<iris_cobot::JointSpeed>("joint_speeds", 1);

    while(true)
    {
        // Goal EE pose
        Eigen::Vector3d translation;
        translation << 0, 0, 0;
        Eigen::Vector3d rotation;
        rotation << 0, 0, 0;

        // Input character for direction
        if (KEYBOARD)
        {
            char input;
            while(1) {
                system("stty raw");
                input = getchar();
                system("stty cooked");
                system("clear");
                if(input == '.') {
                    system("stty cooked");
                    exit(0);
                }
                else
                {
                    if(input == 'w')
                    {
                        translation[0] = 0.2;
                        break;
                    }
                    else if(input == 's')
                    {
                        translation[0] = -0.2;
                        break;
                    }
                    else if(input == 'a')
                    {
                        translation[1] = 0.2;
                        break;
                    }
                    else if(input == 'd')
                    {
                        translation[1] = -0.2;
                        break;
                    }
                    else if(input == 'u')
                    {
                        rotation[0] = 0.2;
                        break;
                    }
                    else if(input == 'j')
                    {
                        rotation[0] = -0.2;
                        break;
                    }
                    else if(input == 'h')
                    {
                        rotation[2] = 0.2;
                        break;
                    }
                    else if(input == 'k')
                    {
                        rotation[2] = -0.2;
                        break;
                    }
                }
            }
        }
        else
        { 
            geometry_msgs::Vector3 linear = *linear_velocity_ptr;
            geometry_msgs::Vector3 angular = *angular_velocity_ptr;
            if (abs(linear.x) > 0.15 || abs(linear.y) > 0.15 || abs(linear.z) > 0.15 ||
                abs(angular.x) > 0.15 || abs(angular.y) > 0.15 || abs(angular.z) > 0.15)
            {
                translation[0] = linear.x;
                translation[1] = linear.y;
                translation[2] = linear.z;
                rotation[0] = angular.x;
                rotation[1] = angular.y;
                rotation[2] = angular.z;
            }
        }

        // Create Kinematic State
        current_joint_values = move_group.getCurrentJointValues();
        kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);

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
    }
    
    ros::shutdown();

    return 0;
}