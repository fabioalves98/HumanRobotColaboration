#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;

pcl::visualization::PCLVisualizer::Ptr viewer;

CloudPtr cloud (new Cloud);

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
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "kinectcloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "kinectcloud");
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    CloudPtr cloud (new Cloud);

    pcl::fromROSMsg(*input, *cloud);

    std::cout << cloud->size() << std::endl;

    viewer->updatePointCloud(cloud, "kinectcloud");
    viewer->spinOnce (100);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "viewer");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

    viewer = normalVis("3DViewer");
    setViewerPointcloud(cloud);

    // Spin
    ros::spin ();
}