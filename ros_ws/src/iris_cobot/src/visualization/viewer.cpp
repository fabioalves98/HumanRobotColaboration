#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;

pcl::visualization::PCLVisualizer::Ptr viewer;

CloudPtr cloud (new Cloud);

bool clustering = false;

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

void drawBoundingBox(Eigen::Vector3f &translation, Eigen::Quaternionf &orientation, double width,
                    double height, double depth, std::string name)
{
    viewer->removeShape(name);
    viewer->addCube(translation, orientation, width, height, depth, name, 0);  
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, name);             
    viewer->setRepresentationToWireframeForAllActors(); 
}

float radians(int degrees)
{
    return (degrees * (M_PI / 180));
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    CloudPtr cloud (new Cloud);

    pcl::fromROSMsg(*input, *cloud);

    // Width - X | Height - Y | Depth - Z
    // Roll - X  | Pitch - Y  | Yaw - Z

    if (clustering)
    {
        // Setting Bounding Box
        float x = 0, y = -0.2, z = 1;
        Eigen::Vector3f translation(x, y, z);
        float roll = radians(-30), pitch = 0, yaw = radians(-3);
        Eigen::Quaternionf rotation = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
                                    * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                                    * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
        float width = 0.5, height = 1.2, depth = 0.4;
        drawBoundingBox(translation, rotation, width, height, depth, "crop");

        CloudPtr cloud_crop (new Cloud);
        
        // Setting Crop Box
        pcl::CropBox<pcl::PointXYZRGB> boxFilter;
        boxFilter.setInputCloud(cloud);
        boxFilter.setTranslation(translation);
        boxFilter.setRotation(Eigen::Vector3f(roll, pitch, yaw));
        boxFilter.setMin(Eigen::Vector4f(-width/2, -height/2, -depth/2, 1.0));
        boxFilter.setMax(Eigen::Vector4f(width/2, height/2, depth/2, 1.0));
        boxFilter.filter(*cloud_crop);

        // Clutering
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        vg.setInputCloud (cloud_crop);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);
        std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
        // pcl::PCDWriter writer;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        int i=0, nr_points = (int) cloud_filtered->size ();
        while (cloud_filtered->size () > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);
            std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            *cloud_filtered = *cloud_f;
        }

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (200);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);


        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            int cluster_size = 0;
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            {
                cloud_filtered->points[*pit].r = 255;
                cloud_filtered->points[*pit].g = 0;
                cloud_filtered->points[*pit].b = 0;
                cluster_size++;
            }

            std::cout << "PointCloud representing the Cluster: " << cluster_size << " data points." << std::endl;
        }
        viewer->updatePointCloud(cloud_filtered, "kinectcloud");

    }
    else
    {
        viewer->updatePointCloud(cloud, "kinectcloud");
    }
    
    viewer->spinOnce(100);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "viewer");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    std::string cloud_topic_name = "/camera/depth/points";
    ros::Subscriber sub = nh.subscribe (cloud_topic_name, 1, cloud_cb);

    viewer = normalVis("3DViewer");
    setViewerPointcloud(cloud);

    // Spin
    ros::spin();
}