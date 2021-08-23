#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <iris_cobot/JointSpeed.h>

std::vector<double> *joint_speeds_ptr;
std::vector<std::vector<double>> *joint_speed_queue_ptr;

int mean_size = 50;

void jointSpeedSub(iris_cobot::JointSpeed msg)
{
    joint_speed_queue_ptr->erase(joint_speed_queue_ptr->begin());
    joint_speed_queue_ptr->push_back(msg.joint_speeds);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_cont");
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    ros::Publisher joint_speed_final = 
        nh.advertise<std_msgs::Float64MultiArray>("/joint_speed_final", 1);

    std::vector<std::vector<double>> joint_speed_queue;
    joint_speed_queue_ptr = &joint_speed_queue;

    // Joint Speed subscriber
    std::vector<double> joint_speeds = {0, 0, 0, 0, 0, 0};
    joint_speeds_ptr = &joint_speeds;
    for(int i = 0; i < mean_size; i++)
    {
        joint_speed_queue.push_back(joint_speeds);
    }
    ros::Subscriber joint_speed_sub = nh.subscribe("joint_speeds", 1, jointSpeedSub);

    // Move it controller
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // TODO: Revisit to see if changing the rate or checking parameters can help
    ros::Rate rate(10);
    while (ros::ok())
    {
        std::vector<double> joint_speed_avrg = {0, 0, 0, 0, 0, 0};

        for (std::vector<double> j_speed : *joint_speed_queue_ptr)
        {
            for (int i = 0; i < 6; i++)
            {
                joint_speed_avrg[i] += j_speed[i];
            }
        }
        for (int i = 0; i < 6; i++)
        {
            joint_speed_avrg[i] /= mean_size;
        }

        std::transform(joint_speed_avrg.begin(), joint_speed_avrg.end(), joint_speed_avrg.begin(), 
                       [](const double joint) { return joint/10;});
        
        std::vector<double> next_joint_values;
        std::vector<double> current_joint_values = move_group.getCurrentJointValues();
        std::transform(current_joint_values.begin(), current_joint_values.end(),
                        joint_speed_avrg.begin(), std::back_inserter(next_joint_values), 
                        std::plus<double>());

        move_group.setJointValueTarget(next_joint_values);
        move_group.asyncMove();

        std_msgs::Float64MultiArray joint_vel_msg;
        joint_vel_msg.data = joint_speed_avrg;
        joint_vel_msg.layout.data_offset = 1;
        joint_speed_final.publish(joint_vel_msg);
        
        rate.sleep();
        // ros::spinOnce();
    }
}