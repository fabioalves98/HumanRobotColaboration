#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <iris_cobot/JointSpeed.h>

std::vector<double> *joint_speeds_ptr;

void jointSpeedSub(iris_cobot::JointSpeed msg)
{
    std::copy(msg.joint_speeds.begin(), msg.joint_speeds.end(), joint_speeds_ptr->begin());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_group_controller");
    ros::NodeHandle nh;

    // Velocity Publisher
    ros::Publisher joint_vel_pub = 
        nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    // Joint Speed subscriber
    std::vector<double> joint_speeds = {0, 0, 0, 0, 0, 0};
    joint_speeds_ptr = &joint_speeds;
    ros::Subscriber joint_speed_sub = nh.subscribe("joint_speeds", 1, jointSpeedSub);

    ros::Rate rate(100);

    while(true)
    {
        std_msgs::Float64MultiArray joint_vel_msg;
        joint_vel_msg.data = *joint_speeds_ptr;
        joint_vel_msg.layout.data_offset = 1;

        joint_vel_pub.publish(joint_vel_msg);
        
        ros::spinOnce();
        rate.sleep();
    }
}