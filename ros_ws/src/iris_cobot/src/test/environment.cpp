#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>


pcl::visualization::PCLVisualizer::Ptr viewer;

typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;

CloudPtr environment (new Cloud);


pcl::visualization::PCLVisualizer::Ptr normalVis(std::string name)
{   
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (name));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
    return viewer;
}

void setViewerPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "camera_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "camera_cloud");
}

bool take_sample(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    CloudPtr cloud_to_add (new Cloud);

    // Wait for cloud message
    boost::shared_ptr<sensor_msgs::PointCloud2 const> cloud_msg_prt;
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg_prt = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/points");
    if (cloud_msg_prt != NULL)
    {
        cloud_msg = *cloud_msg_prt;
    }
    pcl::fromROSMsg(cloud_msg, *cloud_to_add);

    // Obtain camera transform
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped tfStamped;
    while (true)
    {
        try
        {
            tfStamped = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
    }

    // Transform point cloud
    Eigen::Affine3d transform =  tf2::transformToEigen(tfStamped);
    pcl::transformPointCloud(*cloud_to_add, *cloud_to_add, transform);

    // Add to point cloud
    *environment += *cloud_to_add;

    // Visualize Point Cloud
    viewer->updatePointCloud(environment, "camera_cloud");
    viewer->spinOnce(100);

    return true;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "environment");
    ros::NodeHandle nh;

    // Concatenation service
    ros::ServiceServer service = nh.advertiseService("iris_cobot/environment/take_sample", take_sample);

    // Setup Viewer
    viewer = normalVis("3DViewer");
    setViewerPointcloud(environment);

    // Viewer Loop
    while (ros::ok())
    {
        viewer->spinOnce(100);
        ros::spinOnce();
    }
}