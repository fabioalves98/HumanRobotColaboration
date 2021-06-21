#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

#include <dynamic_reconfigure/server.h>
#include <iris_cobot/FT_to_VelConfig.h>

#include <moveit/move_group_interface/move_group_interface.h>


moveit::planning_interface::MoveGroupInterface *move_group_ptr;
tf2_ros::TransformBroadcaster *br_ptr;
ros::Publisher *force_marker_pub_ptr;
ros::Publisher *ang_vel_pub_ptr;
ros::Publisher *lin_vel_pub_ptr;

double force_div;
double torque_div;

void parameterConfigure(iris_cobot::FT_to_VelConfig &config, uint32_t level) 
{
    force_div = config.force_div;
    torque_div = config.torque_div;
}


void rotationCalculator(geometry_msgs::WrenchStamped wrench)
{
    geometry_msgs::Vector3 force = wrench.wrench.force;
    geometry_msgs::Vector3 torque = wrench.wrench.torque;

    // Divide force and torque values by a user defined constant
    force.x /= force_div;
    force.y /= force_div;
    force.z /= force_div;

    torque.x /= torque_div;
    torque.y /= torque_div;
    torque.z /= torque_div;

    // Origin position
    tf2::Vector3 origin(-0.7, 0.3, 0.5);

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
    force_pose.position.x = force.x;
    force_pose.position.y = force.y;
    force_pose.position.z = force.z;
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

    // Difference between poses
    geometry_msgs::Vector3 lin_vel;
    lin_vel.x = force_pose.position.x - origin.getX();
    lin_vel.y = force_pose.position.y - origin.getY();
    lin_vel.z = force_pose.position.z - origin.getZ();

    // Publish Difference between poses - linear velocity
    lin_vel_pub_ptr->publish(lin_vel);

    // TORQUE
    // Create Torque Orientation
    tf2::Quaternion torque_rot, torque_ori;
    torque_rot.setRPY(torque.x, torque.y, torque.z);
    torque_ori = ft_sensor_ori * torque_rot;

    // Torque Transform
    tf2::Transform torque_tf;
    torque_tf.setRotation(torque_ori);

    // Publish Torque Transform
    geometry_msgs::TransformStamped torque_tf_stamped;
    torque_tf_stamped.header.stamp = ros::Time::now();
    torque_tf_stamped.header.frame_id = "base_link";
    torque_tf_stamped.child_frame_id = "torque";
    torque_tf_stamped.transform.translation.x = -0.7;
    torque_tf_stamped.transform.translation.y = 0.3;
    torque_tf_stamped.transform.translation.z = 0.5;    
    torque_tf_stamped.transform.rotation = tf2::toMsg(torque_tf.getRotation());

    br_ptr->sendTransform(torque_tf_stamped);

    // Diference between transforms
    tf2::Transform diff_tf;
    double roll, pitch, yaw;
    diff_tf = torque_tf * ft_sensor_tf.inverse();
    tf2::Matrix3x3(diff_tf.getRotation()).getRPY(roll, pitch, yaw);

    // Publish Difference between transforms - angular velocity
    geometry_msgs::Vector3 angvel;
    angvel.x = roll;
    angvel.y = pitch;
    angvel.z = yaw;

    ang_vel_pub_ptr->publish(angvel);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ft_to_vel");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group_ptr = &move_group;

    ros::Publisher force_marker_pub = nh.advertise<visualization_msgs::Marker>("f_marker", 1);
    force_marker_pub_ptr = &force_marker_pub;

    tf2_ros::TransformBroadcaster br;
    br_ptr = &br;

    ros::Publisher lin_vel_pub = nh.advertise<geometry_msgs::Vector3>("linear_velocity", 1);
    lin_vel_pub_ptr = &lin_vel_pub;

    ros::Publisher ang_vel_pub = nh.advertise<geometry_msgs::Vector3>("angular_velocity", 1);
    ang_vel_pub_ptr = &ang_vel_pub;

    // Dynamic reconfigure init and callback
    dynamic_reconfigure::Server<iris_cobot::FT_to_VelConfig> server;
    dynamic_reconfigure::Server<iris_cobot::FT_to_VelConfig>::CallbackType cobotConfigCallback;
    cobotConfigCallback = boost::bind(&parameterConfigure, _1, _2);
    server.setCallback(cobotConfigCallback);

    ros::Subscriber wrench_sub = nh.subscribe("wrench_correct", 1, rotationCalculator);

    ros::waitForShutdown();

    return 0;
};