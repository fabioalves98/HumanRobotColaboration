#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// PCL specific includes
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <dynamic_reconfigure/server.h>
#include <iris_cobot/ClusteringConfig.h>

#include <iris_cobot/Obstacles.h>


pcl::visualization::PCLVisualizer::Ptr viewer;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointCloud<PointNormal> CloudNormal;
typedef CloudNormal::Ptr CloudNormalPtr;
typedef pcl::IndicesClusters IdxClusters;
typedef pcl::IndicesClustersPtr IdxClustersPtr;

// Global point cloud from camera
CloudPtr cloud_global (new Cloud);

// Obstacles publisher
ros::Publisher obstacles_pub;

// Parameters
double leaf_size;
double radius_search;
double cluster_tolerance;
double min_cluster_size, max_cluster_size;
double squared_dist;
double normal_diff;
double min_point_matrix_det;

void parameterConfigure(iris_cobot::ClusteringConfig &config, uint32_t level) 
{
    leaf_size = config.leaf_size;
    radius_search = config.radius_search;
    cluster_tolerance = config.cluster_tolerance;
    min_cluster_size = config.min_cluster_size;
    max_cluster_size = config.max_cluster_size;
    squared_dist = config.squared_dist;
    normal_diff = config.normal_diff;
    min_point_matrix_det = config.min_point_matrix_det;
}

pcl::visualization::PCLVisualizer::Ptr normalVis(std::string name)
{   
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (name));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
    return viewer;
}

void setViewerPointcloud(pcl::PointCloud<Point>::ConstPtr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(cloud);
    viewer->addPointCloud<Point> (cloud, rgb, "camera_cloud");
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

double determinant4x4(double matrix[4][4])
{
    double c, det = 1;
    for(int i = 0; i < 4; i++) 
    {
        for(int k = i+1; k < 4; k++) 
        {
            c = matrix[k][i] / matrix[i][i];
            for(int j = i; j < 4; j++)
            {
                matrix[k][j] = matrix[k][j] - c*matrix[i][j];
            }
        }
    }
    for (int i = 0; i < 4; i++)
    {
        det *= matrix[i][i];
    }
    return det;
}

bool customRegionGrowing (const PointNormal& point_a, const PointNormal& point_b, float squared_distance)
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
    pcl::console::TicToc tt;
    tt.tic();

    // Convert cloud msg to pcl point cloud
    pcl::fromROSMsg (*cloud_msg, *cloud_global);

    // Data containers used
    CloudPtr cloud_out (new Cloud);
    CloudNormalPtr cloud_with_normals (new CloudNormal);
    IdxClustersPtr clusters (new IdxClusters), small_clusters (new IdxClusters), large_clusters (new IdxClusters);
    
    pcl::search::KdTree<Point>::Ptr search_tree (new pcl::search::KdTree<Point>);
    
    // Downsample the cloud using a Voxel Grid class
    // std::cerr << "Downsampling...\n", tt.tic ();
    pcl::VoxelGrid<Point> vg;
    vg.setInputCloud (cloud_global);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.setDownsampleAllData (true);
    vg.filter (*cloud_out);
    // std::cerr << ">> Done: " << tt.toc() << " ms, " << cloud_out->size () << " points\n";

    // Set up a Normal Estimation class and merge data in cloud_with_normals
    // std::cerr << "Computing normals...\n", tt.tic ();
    pcl::copyPointCloud (*cloud_out, *cloud_with_normals);
    pcl::NormalEstimation<Point, PointNormal> ne;
    ne.setInputCloud (cloud_out);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*cloud_with_normals);
    // std::cerr << ">> Done: " << tt.toc() << " ms\n";

    // Set up a Conditional Euclidean Clustering class
    // std::cerr << "Segmenting to clusters...\n", tt.tic ();
    pcl::ConditionalEuclideanClustering<PointNormal> cec (true);
    cec.setInputCloud (cloud_with_normals);
    cec.setConditionFunction (&customRegionGrowing);
    cec.setClusterTolerance (cluster_tolerance);
    cec.setMinClusterSize (10);
    cec.setMaxClusterSize (cloud_with_normals->points.size() * max_cluster_size / 100);
    cec.segment (*clusters);
    cec.getRemovedClusters (small_clusters, large_clusters);
    // std::cerr << ">> Done: " << tt.toc () << " ms\n";
    // std::cout << "Clusters Size: " << clusters->size() << "\n\n";

    // Paint Pointclouds
    paintFullCluster(cloud_out, small_clusters, 0, 100, 0);
    paintFullCluster(cloud_out, large_clusters, 0, 0, 100);
    paintFullCluster(cloud_out, clusters, 255, 0, 0);
    // paintRandomCluster(cloud_out, clusters);

    // Obstacles arrays
    std::vector<geometry_msgs::Point> obstacles_centers;
    std::vector<double> obstacles_radiuses;

    // Select 4 random points from each cluster
    for (int i = 0; i < clusters->size(); i++)
    {
        std::cout << "Cluster " << i << "\n";

        // Selecting 4 random points in the cluster, verify if they have volume (not a plane)
        std::vector<int> random_indices;
        for (int j = 0; j < 4; j++)
        {   
            int index = rand() % clusters->at(i).indices.size();
            if (std::find(random_indices.begin(), random_indices.end(), index) != random_indices.end())
            {
                j--;
                continue;
            }
            random_indices.push_back(index);
        }

        double point_matrix[4][4];
        int pm_idx = 0;
        for (int idx : random_indices)
        {
            cloud_out->points[clusters->at(i).indices[idx]].r = 255;
            cloud_out->points[clusters->at(i).indices[idx]].g = 255;
            cloud_out->points[clusters->at(i).indices[idx]].b = 255;

            Point point = cloud_out->points[clusters->at(i).indices[idx]];
            point_matrix[pm_idx][0] = point.x;
            point_matrix[pm_idx][1] = point.y;
            point_matrix[pm_idx][2] = point.z;
            point_matrix[pm_idx][3] = 1;
            pm_idx++;
        }
        // Only run RUNSAC for clusters that have volume - det > 0
        std::cout << "Determinant - " << determinant4x4(point_matrix) << "\n";

        // TODO: Obtain center and radius using the 4 extracted points

        // Extract obstacle from cloud
        CloudPtr obstacle (new Cloud);
        pcl::copyPointCloud(*cloud_out, *obstacle);
        pcl::PointIndices::Ptr filter_indices (new pcl::PointIndices);
        filter_indices->indices = clusters->at(i).indices;

        pcl::ExtractIndices<Point> filter;
        filter.setInputCloud (cloud_out);
        filter.setIndices(filter_indices);
        filter.filter(*obstacle);

        // Run RANSAC model for sphere 
        pcl::SampleConsensusModelSphere<Point>::Ptr model_s(new pcl::SampleConsensusModelSphere<Point> (obstacle));
        pcl::RandomSampleConsensus<Point> ransac (model_s);
        ransac.setDistanceThreshold(0.01);
        ransac.computeModel();
        Eigen::VectorXf model_coefficients;
        ransac.getModelCoefficients(model_coefficients);

        // TODO: CHANGE THIS - Fastest way to determine if it's a sphere obstacle or not
        if (model_coefficients[3] < 0.06)
        {
            // Add obstacle to obstacle array msgs
            geometry_msgs::Point center;
            center.x = model_coefficients[0];
            center.y = model_coefficients[1];
            center.z = model_coefficients[2];
            obstacles_centers.push_back(center);
            obstacles_radiuses.push_back(model_coefficients[3]);

            std::cout << "Coeficients - X: " << model_coefficients[0] << ", Y: " << model_coefficients[2];
            std::cout << ", Z: " << model_coefficients[2] << ", R: " << model_coefficients[3] << "\n";
        }
    }
    std::cout << std::endl;

    // Publish obstacles message
    iris_cobot::Obstacles obstacles_msg;
    obstacles_msg.size = obstacles_centers.size();
    obstacles_msg.centers = obstacles_centers;
    obstacles_msg.radiuses = obstacles_radiuses;
    obstacles_pub.publish(obstacles_msg);

    tt.toc_print();

    // Show obstacles in PCL visualizer
    viewer->updatePointCloud(cloud_out, "camera_cloud");
    viewer->spinOnce();
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

    // Obstacles Publisher
    obstacles_pub = nh.advertise<iris_cobot::Obstacles>("obstacles", 1);

    // Setup Viewer
    viewer = normalVis("3DViewer");
    setViewerPointcloud(cloud_global);

    // Spin
    ros::spin();
}








