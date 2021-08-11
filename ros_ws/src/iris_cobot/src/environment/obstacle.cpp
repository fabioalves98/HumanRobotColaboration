#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <dynamic_reconfigure/server.h>
#include <iris_cobot/ClusteringConfig.h>

pcl::visualization::PCLVisualizer::Ptr viewer;

typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudNormal;
typedef CloudNormal::Ptr CloudNormalPtr;
typedef pcl::IndicesClusters IdxClusters;
typedef pcl::IndicesClustersPtr IdxClustersPtr;

// Global point cloud from camera
CloudPtr cloud_global (new Cloud);

// Parameters
double leaf_size;
double radius_search;
double cluster_tolerance;
int min_cluster_size, max_cluster_size;
double squared_dist;
double normal_diff;

void parameterConfigure(iris_cobot::ClusteringConfig &config, uint32_t level) 
{
    leaf_size = config.leaf_size;
    radius_search = config.radius_search;
    cluster_tolerance = config.cluster_tolerance;
    min_cluster_size = config.min_cluster_size;
    max_cluster_size = config.max_cluster_size;
    squared_dist = config.squared_dist;
    normal_diff = config.normal_diff;
}

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

void paintFullCluster(CloudPtr cloud, IdxClustersPtr clusters, int r, int g, int b)
{
    for (int i = 0; i < clusters->size(); i++)
    {
        for (int j = 0; j < (*clusters)[i].indices.size(); j++)
        {
            cloud->points[(*clusters)[i].indices[j]].r = r;
            cloud->points[(*clusters)[i].indices[j]].g = g;
            cloud->points[(*clusters)[i].indices[j]].b = b;
        }
    }
}

void paintRandomCluster(CloudPtr cloud, IdxClustersPtr clusters)
{
    for (int i = 0; i < clusters->size(); i++)
    {
        int r = 55 + rand() % 200;
        int g = 55 + rand() % 200;
        int b = 55 + rand() % 200;

        for (int j = 0; j < (*clusters)[i].indices.size(); j++)
        {
            cloud->points[(*clusters)[i].indices[j]].r = r;
            cloud->points[(*clusters)[i].indices[j]].g = g;
            cloud->points[(*clusters)[i].indices[j]].b = b;
        }
    }
}

bool customRegionGrowing (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), 
    point_b_normal = point_b.getNormalVector3fMap ();
    double enf_normal_diff = point_a_normal.dot(point_b_normal);
    
    if (squared_distance < squared_dist)
    {
        if (enf_normal_diff < normal_diff)
        {
            return (true);  
        }
    }

    return (false);
}

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::fromROSMsg (*cloud_msg, *cloud_global);

    // CLUSTERING

    // Data containers used
    CloudPtr cloud_out (new Cloud);
    CloudNormalPtr cloud_with_normals (new CloudNormal);
    IdxClustersPtr clusters (new IdxClusters), small_clusters (new IdxClusters), large_clusters (new IdxClusters);
    
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::console::TicToc tt;

    // Downsample the cloud using a Voxel Grid class
    std::cerr << "Downsampling...\n", tt.tic ();
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_global);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.setDownsampleAllData (true);
    vg.filter (*cloud_out);
    std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_out->size () << " points\n";

    // Set up a Normal Estimation class and merge data in cloud_with_normals
    std::cerr << "Computing normals...\n", tt.tic ();
    pcl::copyPointCloud (*cloud_out, *cloud_with_normals);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud (cloud_out);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*cloud_with_normals);
    std::cerr << ">> Done: " << tt.toc () << " ms\n";

    // Set up a Conditional Euclidean Clustering class
    std::cerr << "Segmenting to clusters...\n", tt.tic ();
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
    cec.setInputCloud (cloud_with_normals);
    cec.setConditionFunction (&customRegionGrowing);
    cec.setClusterTolerance (cluster_tolerance);
    cec.setMinClusterSize (cloud_with_normals->points.size () * min_cluster_size / 100);
    cec.setMaxClusterSize (cloud_with_normals->points.size () * max_cluster_size / 100);
    cec.segment (*clusters);
    cec.getRemovedClusters (small_clusters, large_clusters);
    std::cerr << ">> Done: " << tt.toc () << " ms\n";
    std::cout << "Clusters Size: " << clusters->size() << "\n\n";

    // Paint Pointclouds
    paintFullCluster(cloud_out, small_clusters, 255, 0, 0);
    paintFullCluster(cloud_out, large_clusters, 0, 0, 255);
    paintRandomCluster(cloud_out, clusters);

    // Select 4 random pointt from each cluster
    for (int i = 0; i < clusters->size(); i++)
    {
        std::vector<int> random_point_indices;
        for (int j = 0; j < 4; j++)
        {
            random_point_indices.push_back(rand() % (*clusters)[i].indices.size());
        }

        for (int idx : random_point_indices)
        {
            cloud_out->points[(*clusters)[i].indices[idx]].r = 255;
            cloud_out->points[(*clusters)[i].indices[idx]].g = 255;
            cloud_out->points[(*clusters)[i].indices[idx]].b = 255;
        }
    }


    viewer->updatePointCloud(cloud_out, "camera_cloud");

    viewer->spinOnce(100);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "obstacle_detection");
    ros::NodeHandle nh;

    // Random seed generator
    srand(time(NULL));

    // Dynamic reconfigure init and callback
    dynamic_reconfigure::Server<iris_cobot::ClusteringConfig> server;
    dynamic_reconfigure::Server<iris_cobot::ClusteringConfig>::CallbackType cobotConfigCallback;
    cobotConfigCallback = boost::bind(&parameterConfigure, _1, _2);
    server.setCallback(cobotConfigCallback);

    // Camera Subscription service
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_callback);

    // Setup Viewer
    viewer = normalVis("3DViewer");
    setViewerPointcloud(cloud_global);

    // Spin
    ros::spin();
}








