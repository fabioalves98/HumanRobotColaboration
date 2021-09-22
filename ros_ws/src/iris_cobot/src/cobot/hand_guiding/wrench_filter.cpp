#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

std::vector<double> *wrench_prev;
ros::Publisher wrench_filtered_pub;

double alpha = 0.05;

void wrenchFilter(geometry_msgs::WrenchStamped msg)
{
    std::vector<double> wrench_cur = {
        msg.wrench.force.x,
        msg.wrench.force.y,
        msg.wrench.force.z,
        msg.wrench.torque.x,
        msg.wrench.torque.y,
        msg.wrench.torque.z,
    };

    std::transform(wrench_prev->begin(), wrench_prev->end(), wrench_cur.begin(), wrench_cur.begin(),
        [](double p, double c)
        { 
            return (1 - alpha) * p + alpha * ((p + c) / 2);
        }
    );

    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.wrench.force.x = wrench_cur[0];
    wrench_msg.wrench.force.y = wrench_cur[1];
    wrench_msg.wrench.force.z = wrench_cur[2];
    wrench_msg.wrench.torque.x = wrench_cur[3];
    wrench_msg.wrench.torque.y = wrench_cur[4];
    wrench_msg.wrench.torque.z = wrench_cur[5];
    wrench_filtered_pub.publish(wrench_msg);

    std::copy(wrench_cur.begin(), wrench_cur.end(), wrench_prev->begin());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrench_filter");
    ros::NodeHandle nh;

    // Wrench Filtered Publisher
    wrench_filtered_pub = nh.advertise<geometry_msgs::WrenchStamped>("wrench_filtered", 1);

    // Dynamic reconfigure init and callback
    // dynamic_reconfigure::Server<iris_cobot::JointFilterConfig> server;
    // dynamic_reconfigure::Server<iris_cobot::JointFilterConfig>::CallbackType cobotConfigCallback;
    // cobotConfigCallback = boost::bind(&parameterConfigure, _1, _2);
    // server.setCallback(cobotConfigCallback);

    wrench_prev = new std::vector<double>();
    wrench_prev->assign(6, 0);
    ros::Subscriber wrench_sub = nh.subscribe("wrench", 1, wrenchFilter);

    ROS_INFO("Wrench filter node node listening to wrench and publishing to wrench_filtered");

    ros::spin();
    
    return 0;
}