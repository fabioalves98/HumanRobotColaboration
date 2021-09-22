#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <dynamic_reconfigure/server.h>
#include <iris_cobot/JointFilterConfig.h>

#include <iris_cobot/JointSpeed.h>

#define SCALLING_FACTOR 5

std::vector<double> *js_cur_p;
std::vector<double> *js_com_p;
std::vector<double> *js_final_p;

double alpha;

bool stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    js_com_p->clear();
}

void parameterConfigure(iris_cobot::JointFilterConfig &config, uint32_t level) 
{
    alpha = config.alpha;
}

void jointSpeedCurrent(sensor_msgs::JointState msg)
{
    std::vector<std::string> names = msg.name;
    if (names.size() != 6)
    {
        return;
    }

    js_cur_p->at(0) = msg.velocity[std::find(names.begin(), names.end(), 
        "shoulder_pan_joint") - names.begin()];
    js_cur_p->at(1) = msg.velocity[std::find(names.begin(), names.end(), 
        "shoulder_lift_joint") - names.begin()];
    js_cur_p->at(2) = msg.velocity[std::find(names.begin(), names.end(), 
        "elbow_joint") - names.begin()];
    js_cur_p->at(3) = msg.velocity[std::find(names.begin(), names.end(), 
        "wrist_1_joint") - names.begin()];
    js_cur_p->at(4) = msg.velocity[std::find(names.begin(), names.end(), 
        "wrist_2_joint") - names.begin()];
    js_cur_p->at(5) = msg.velocity[std::find(names.begin(), names.end(), 
        "wrist_3_joint") - names.begin()];

    std::transform(js_cur_p->begin(), js_cur_p->end(), js_cur_p->begin(), 
        [](double e) { return e * SCALLING_FACTOR; });
}

void jointSpeedCommand(iris_cobot::JointSpeed msg)
{
    std::copy(msg.joint_speeds.begin(), msg.joint_speeds.end(), js_com_p->begin());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_vel_filter");
    ros::NodeHandle nh;

    // Velocity Publisher
    ros::Publisher joint_vel_pub = 
        nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    // Error Publisher
    ros::Publisher joint_speed_final = 
        nh.advertise<std_msgs::Float64MultiArray>("/joint_speed_final", 1);
    
    // Joint Speed Command
    std::vector<double> joint_speed_command = {0, 0, 0, 0, 0, 0};
    js_com_p = &joint_speed_command;

    // Current Joint State
    std::vector<double> joint_speed_current = {0, 0, 0, 0, 0, 0};
    js_cur_p = &joint_speed_current;

    // Dynamic reconfigure init and callback
    dynamic_reconfigure::Server<iris_cobot::JointFilterConfig> server;
    dynamic_reconfigure::Server<iris_cobot::JointFilterConfig>::CallbackType cobotConfigCallback;
    cobotConfigCallback = boost::bind(&parameterConfigure, _1, _2);
    server.setCallback(cobotConfigCallback);

    ros::Subscriber joint_state_sub = nh.subscribe("joint_states", 1, jointSpeedCurrent);
    ros::Subscriber joint_speed_sub = nh.subscribe("joint_speeds", 1, jointSpeedCommand);

    // Joint Speed control
    ros::ServiceServer service = nh.advertiseService("joint_speeds/stop", stop);

    ROS_INFO("Joint Speed Controller node listening to joint_speeds and publishing to UR Driver");

    std::vector<double> prev = {0, 0, 0, 0, 0, 0};

    ros::Rate rate(500);
    while(ros::ok())
    {
        std::vector<double> new_vel = {0, 0, 0 ,0 ,0, 0};
        std::transform(prev.begin(), prev.end(), js_com_p->begin(), new_vel.begin(),
            [](double p, double c)
            { 
                return (1 - alpha) * p + alpha * ((p + c) / 2);
            }
        );
        
        std_msgs::Float64MultiArray joint_vel_msg;
        joint_vel_msg.data = new_vel;
        joint_vel_msg.layout.data_offset = 1;

        joint_speed_final.publish(joint_vel_msg);
        joint_vel_pub.publish(joint_vel_msg);

        std::copy(new_vel.begin(), new_vel.end(), prev.begin());
        
        ros::spinOnce();
        rate.sleep();
    }
}