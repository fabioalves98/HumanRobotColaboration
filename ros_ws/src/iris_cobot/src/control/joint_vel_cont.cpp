#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

#include <iris_cobot/JointSpeed.h>

std::vector<double> *joint_speeds_ptr;
std::vector<std::vector<double>> *joint_speed_queue_ptr;

int mean_size = 50;

bool stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    for (int i = 0; i < mean_size; i++)
    {
        std::vector<double> stopped =  {0, 0, 0, 0, 0, 0};
        joint_speed_queue_ptr->erase(joint_speed_queue_ptr->begin());
        joint_speed_queue_ptr->push_back(stopped);
    }

    return true;
}

void jointSpeedSub(iris_cobot::JointSpeed msg)
{
    joint_speed_queue_ptr->erase(joint_speed_queue_ptr->begin());
    joint_speed_queue_ptr->push_back(msg.joint_speeds);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_vel_cont");
    ros::NodeHandle nh;

    // Velocity Publisher
    ros::Publisher joint_vel_pub = 
        nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);
    
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

    // Joint Speed control
    ros::ServiceServer service = nh.advertiseService("joint_speeds/stop", stop);

    ros::Rate rate(500);
    while(ros::ok())
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

        std_msgs::Float64MultiArray joint_vel_msg;
        joint_vel_msg.data = joint_speed_avrg;
        joint_vel_msg.layout.data_offset = 1;

        joint_vel_pub.publish(joint_vel_msg);
        joint_speed_final.publish(joint_vel_msg);
        
        ros::spinOnce();
        rate.sleep();
    }
}