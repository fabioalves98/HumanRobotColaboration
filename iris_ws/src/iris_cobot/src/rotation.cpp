#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok())
  {
    tf::StampedTransform ft_sensor_tf;
    try{
      listener.lookupTransform("/world", "/ft_sensor",  
                               ros::Time(0), ft_sensor_tf);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf::StampedTransform torque_tf;
    try{
      listener.lookupTransform("/world", "/torque",  
                               ros::Time(0), torque_tf);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf::Transform diff_tf = ft_sensor_tf.inverseTimes(torque_tf);

    double roll, pitch, yaw;
    tf::Matrix3x3(diff_tf.getRotation()).getRPY(roll, pitch, yaw);

    std::cout << roll << " - " << pitch << " - " << yaw << std::endl;

    rate.sleep();
  }
  return 0;
};