#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <iris_cobot/JointSpeed.h>

std::vector<double> *joint_speeds_ptr;

void jointSpeedSub(iris_cobot::JointSpeed msg)
{
    std::copy(msg.joint_speeds.begin(), msg.joint_speeds.end(), joint_speeds_ptr->begin());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_cont");
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    // Joint Speed subscriber
    std::vector<double> joint_speeds = {0, 0, 0, 0, 0, 0};
    joint_speeds_ptr = &joint_speeds;
    ros::Subscriber joint_speed_sub = nh.subscribe("joint_speeds", 1, jointSpeedSub);

    // Move it controller
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    ROS_INFO("MoveIt controller node listening to joint_speeds and controlling move_group interface");

    ros::Rate rate(10);
    // TODO: Revisit to see if changing the rate or checking parameters can help
    while (ros::ok())
    {
        std::vector<double> joint_speeds_inc = *joint_speeds_ptr;

        std::transform(joint_speeds_inc.begin(), joint_speeds_inc.end(), joint_speeds_inc.begin(), 
                       [](const double joint) { return joint/10;});
        
        std::vector<double> next_joint_values;
        std::vector<double> current_joint_values = move_group.getCurrentJointValues();
        std::transform(current_joint_values.begin(), current_joint_values.end(),
                        joint_speeds_inc.begin(), std::back_inserter(next_joint_values), 
                        std::plus<double>());
        
        move_group.setJointValueTarget(next_joint_values);
        move_group.asyncMove();

        rate.sleep();
    }
    ros::waitForShutdown();
}