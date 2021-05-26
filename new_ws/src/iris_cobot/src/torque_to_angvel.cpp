#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <moveit/move_group_interface/move_group_interface.h>


moveit::planning_interface::MoveGroupInterface *move_group_ptr;
tf2_ros::TransformBroadcaster *br_ptr;
ros::Publisher *ang_vel_pub_ptr;

void rotationCalculator(geometry_msgs::WrenchStamped wrench)
{
    geometry_msgs::Vector3 torque = wrench.wrench.torque;

    // Temporary limit and contrain torque sensibility
    torque.x /= 10;
    torque.y /= 10;
    torque.z /= 10;
    if (torque.x > M_PI_4/2) torque.x = M_PI_4/2;
    if (torque.x < -M_PI_4/2) torque.x = -M_PI_4/2;
    if (torque.y > M_PI_4/2) torque.y = M_PI_4/2;
    if (torque.y < -M_PI_4/2) torque.y = -M_PI_4/2;
    if (torque.z > M_PI_4/2) torque.z = M_PI_4/2;
    if (torque.z < -M_PI_4/2) torque.z = -M_PI_4/2;

    // Create FT Sensor Orientation
    geometry_msgs::PoseStamped ee_pose = move_group_ptr->getCurrentPose();
    tf2::Quaternion ee_ori, ft_sensor_rot, ft_sensor_ori;
    tf2::convert(ee_pose.pose.orientation, ee_ori);
    ft_sensor_rot.setRPY(-M_PI_2, 0, -M_PI_2);
    ft_sensor_ori = ee_ori * ft_sensor_rot;

    // Create Torque Orientation
    tf2::Quaternion torque_rot, torque_ori;
    torque_rot.setRPY(torque.x, torque.y, torque.z);
    torque_ori = ft_sensor_ori * torque_rot;

    // FT Sensor Transform
    tf2::Transform ft_sensor_tf;
    ft_sensor_tf.setRotation(ft_sensor_ori);   

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
    // std::cout << roll << " | " << pitch << " | " << yaw << std::endl;

    // Publish Difference between transforms - angular velocity
    geometry_msgs::Vector3 angvel;
    angvel.x = roll;
    angvel.y = pitch;
    angvel.z = yaw;

    ang_vel_pub_ptr->publish(angvel);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "torque_to_angvel");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group_ptr = &move_group;

    tf2_ros::TransformBroadcaster br;
    br_ptr = &br;

    ros::Publisher angvel_pub = nh.advertise<geometry_msgs::Vector3>("angular_velocity", 1);
    ang_vel_pub_ptr = &angvel_pub;

    ros::Subscriber wrench_sub = nh.subscribe("wrench", 1, rotationCalculator);

    ros::waitForShutdown();

    return 0;
};