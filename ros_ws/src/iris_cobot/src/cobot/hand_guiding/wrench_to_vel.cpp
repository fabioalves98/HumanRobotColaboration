#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

#include <dynamic_reconfigure/server.h>
#include <iris_cobot/FTtoVelConfig.h>
#include <iris_cobot/WrenchControl.h>

#include <moveit/move_group_interface/move_group_interface.h>

moveit::planning_interface::MoveGroupInterface *move_group_ptr;
tf2_ros::TransformBroadcaster *br_ptr;
ros::Publisher *force_marker_pub_ptr;
ros::Publisher *wrench_vel_pub_ptr;

double force_div;
double torque_div;
double force_sensibility;
double torque_sensibility;

std::string status = "stop";
std::vector<std::string> statuses = {"play", "pause", "stop"};

bool hg_control(iris_cobot::WrenchControl::Request& req, iris_cobot::WrenchControl::Response& res)
{
    for (std::string status_s : statuses)
    {
        if (status_s == req.control)
        {
            status = req.control;
            res.success = true;
            return true;
        }
    }

    res.success = false;
    return false;
}

void parameterConfigure(iris_cobot::FTtoVelConfig &config, uint32_t level) 
{
    force_div = config.force_div;
    torque_div = config.torque_div;

    force_sensibility = config.force_sensibility;
    torque_sensibility = config.torque_sensibility;
}

void rotationCalculator(geometry_msgs::WrenchStamped wrench)
{
    // Obtain values from msg
    geometry_msgs::Vector3 force_msg = wrench.wrench.force;
    geometry_msgs::Vector3 torque_msg = wrench.wrench.torque;
    Eigen::Vector3d force;
    Eigen::Vector3d torque;
    tf::vectorMsgToEigen(force_msg, force);
    tf::vectorMsgToEigen(torque_msg, torque);

    // Limit force and torque sensibility by a defined threshold
    std::vector<std::pair<Eigen::Vector3d*, double>> wrench_map = {{&force, force_sensibility}, 
                                                                   {&torque, torque_sensibility}};

    for (auto &pair : wrench_map)
    {
        for (int i = 0; i < 3; i++)
        {
            if ((*pair.first)[i] > pair.second)
            {
                (*pair.first)[i] -= pair.second;
            }
            else if ((*pair.first)[i] < -pair.second)
            {
                (*pair.first)[i] += pair.second;
            }
            else
            {
                (*pair.first)[i] = 0;
            }
        }
    }

    // Divide force and torque values by a user defined constant
    force /= force_div;
    torque /= torque_div;
    
    // Origin position
    tf2::Vector3 origin(-0.3, -1.0, 0.7);

    // Create FT Sensor Orientation
    geometry_msgs::PoseStamped ee_pose = move_group_ptr->getCurrentPose();
    tf2::Quaternion ee_ori, ft_sensor_rot, ft_sensor_ori;
    tf2::convert(ee_pose.pose.orientation, ee_ori);
    ft_sensor_rot.setRPY(-M_PI_2, 0, -M_PI_2);
    ft_sensor_ori = ee_ori * ft_sensor_rot;

    // FT Sensor Transform
    tf2::Stamped<tf2::Transform>  ft_sensor_tf;
    ft_sensor_tf.setOrigin(origin);
    ft_sensor_tf.setRotation(ft_sensor_ori);

    // FORCE
    // Create Force Pose
    geometry_msgs::Pose force_pose;
    tf::pointEigenToMsg(force, force_pose.position);
    force_pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

    tf2::doTransform(force_pose, force_pose, tf2::toMsg(ft_sensor_tf));

    // Publish Force Pose
    visualization_msgs::Marker force_marker;
    force_marker.header.frame_id = "base_link";
    force_marker.type = force_marker.SPHERE;
    force_marker.action = force_marker.ADD;
    force_marker.scale = tf2::toMsg(tf2::Vector3(0.03, 0.03, 0.03));
    force_marker.pose = force_pose;
    force_marker.color.r = 1;
    force_marker.color.g = 1;
    force_marker.color.b = 1;
    force_marker.color.a = 1;

    force_marker_pub_ptr->publish(force_marker);

    // Create Wrench Velocity Message
    geometry_msgs::WrenchStamped wrench_velocity_msg;

    // Difference between poses
    wrench_velocity_msg.wrench.force.x = force_pose.position.x - origin.getX();
    wrench_velocity_msg.wrench.force.y = force_pose.position.y - origin.getY();
    wrench_velocity_msg.wrench.force.z = force_pose.position.z - origin.getZ();

    // TORQUE
    // Create Torque Orientation
    tf2::Quaternion torque_rot, torque_ori;
    torque_rot.setRPY(torque[0], torque[1], torque[2]);
    torque_ori = ft_sensor_ori * torque_rot;

    // Torque Transform
    tf2::Transform torque_tf;
    torque_tf.setRotation(torque_ori);

    // Publish Torque Transform
    geometry_msgs::TransformStamped torque_tf_stamped;
    torque_tf_stamped.header.stamp = ros::Time::now();
    torque_tf_stamped.header.frame_id = "base_link";
    torque_tf_stamped.child_frame_id = "torque";
    torque_tf_stamped.transform.translation.x = origin.x();
    torque_tf_stamped.transform.translation.y = origin.y();
    torque_tf_stamped.transform.translation.z = origin.z();    
    torque_tf_stamped.transform.rotation = tf2::toMsg(torque_tf.getRotation());

    br_ptr->sendTransform(torque_tf_stamped);

    // Diference between transforms
    tf2::Transform diff_tf;
    double roll, pitch, yaw;
    diff_tf = torque_tf * ft_sensor_tf.inverse();
    tf2::Matrix3x3(diff_tf.getRotation()).getRPY(roll, pitch, yaw);

    wrench_velocity_msg.wrench.torque.x = roll;
    wrench_velocity_msg.wrench.torque.y = pitch;
    wrench_velocity_msg.wrench.torque.z = yaw;

    // Publish wrench velocity message
    if (status == "play")
    {
        wrench_vel_pub_ptr->publish(wrench_velocity_msg);
    }
    else if (status == "pause")
    {
        wrench_vel_pub_ptr->publish(geometry_msgs::WrenchStamped());
    }
    else if (status == "stop")
    {

    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "wrench_to_vel");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group_ptr = &move_group;

    ros::Publisher force_marker_pub = nh.advertise<visualization_msgs::Marker>("f_marker", 1);
    force_marker_pub_ptr = &force_marker_pub;

    tf2_ros::TransformBroadcaster br;
    br_ptr = &br;

    ros::Publisher wrench_vel_pub = nh.advertise<geometry_msgs::WrenchStamped>("wrench_velocity", 1);
    wrench_vel_pub_ptr = &wrench_vel_pub;

    // Dynamic reconfigure init and callback
    dynamic_reconfigure::Server<iris_cobot::FTtoVelConfig> server;
    dynamic_reconfigure::Server<iris_cobot::FTtoVelConfig>::CallbackType cobotConfigCallback;
    cobotConfigCallback = boost::bind(&parameterConfigure, _1, _2);
    server.setCallback(cobotConfigCallback);

    // Hand Guiding control service
    ros::ServiceServer service = nh.advertiseService("hg_control", hg_control);

    ros::Subscriber wrench_sub = nh.subscribe("wrench_correct", 1, rotationCalculator);

    ros::waitForShutdown();

    return 0;
}