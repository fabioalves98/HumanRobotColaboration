#include <math.h> 
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
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <dynamic_reconfigure/server.h>
#include <iris_cobot/ObstacleDetectionConfig.h>

#include <iris_cobot/Obstacles.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointCloud<PointNormal> CloudNormal;
typedef CloudNormal::Ptr CloudNormalPtr;
typedef pcl::IndicesClusters IdxClusters;
typedef pcl::IndicesClustersPtr IdxClustersPtr;

// PCL Viewers
pcl::visualization::PCLVisualizer::Ptr viewer_viz;

// Global point cloud from camera
CloudPtr cloud_global (new Cloud);

// Transform listener buffer
tf2_ros::Buffer tfBuffer;

// Camera fixed tf
tf2::Stamped<tf2::Transform> camera_tf;

// Obstacles publisher
ros::Publisher obstacles_pub;

// Parameters
bool viz_cloud;
bool robot_models;
bool obstacle_models;
bool random_obstacles;
bool sphere_obstacles;
double min_dist_robot;
double min_dist_interest;
double leaf_size;
double radius_search;
double cluster_tolerance;
double min_cluster_size;
double max_cluster_size;
double squared_dist;
double normal_diff;

void parameterConfigure(iris_cobot::ObstacleDetectionConfig &config, uint32_t level) 
{
    viz_cloud = config.viz_cloud;
    robot_models = config.robot_models;
    obstacle_models = config.obstacle_models;
    random_obstacles = config.random_obstacles;
    sphere_obstacles = config.sphere_obstacles;
    min_dist_robot = config.min_dist_robot;
    min_dist_interest = config.min_dist_interest;
    leaf_size = config.leaf_size;
    radius_search = config.radius_search;
    cluster_tolerance = config.cluster_tolerance;
    min_cluster_size = config.min_cluster_size;
    max_cluster_size = config.max_cluster_size;
    squared_dist = config.squared_dist;
    normal_diff = config.normal_diff;
}

pcl::visualization::PCLVisualizer::Ptr normalVis(CloudPtr cloud, std::string name, std::string id)
{   
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (name));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
    pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(cloud);
    viewer->addPointCloud<Point> (cloud, rgb, id);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
    return viewer;
}

void paintFullCluster(int index, CloudPtr cloud, IdxClustersPtr clusters, int r, int g, int b)
{
    if (index == -1)
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
    else
    {
        for (int j = 0; j < (*clusters)[index].indices.size(); j++)
            {
                cloud->points[(*clusters)[index].indices[j]].r = r;
                cloud->points[(*clusters)[index].indices[j]].g = g;
                cloud->points[(*clusters)[index].indices[j]].b = b;
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

    // Clean viewers
    viewer_viz->removeAllShapes();

    // Construct robot model for self-identification
    geometry_msgs::TransformStamped sl_tf_msg, ua_tf_msg, fa_tf_msg, w1_tf_msg, w2_tf_msg;
    try
    {
        sl_tf_msg = tfBuffer.lookupTransform("base_link", "shoulder_link", ros::Time(0));
        fa_tf_msg = tfBuffer.lookupTransform("base_link", "forearm_link", ros::Time(0));
        w1_tf_msg = tfBuffer.lookupTransform("base_link", "wrist_1_link", ros::Time(0));
        w2_tf_msg = tfBuffer.lookupTransform("base_link", "wrist_2_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }

    tf2::Stamped<tf2::Transform> sl_tf, fa_tf, w1_tf, w2_tf;
    tf2::fromMsg(sl_tf_msg, sl_tf);
    tf2::fromMsg(fa_tf_msg, fa_tf);
    tf2::fromMsg(w1_tf_msg, w1_tf);
    tf2::fromMsg(w2_tf_msg, w2_tf);

    tf2::Transform offset;
    tf2::Transform world_camera_tf = camera_tf.inverse();
    tf2::Transform sl_camera_tf = camera_tf.inverse() * sl_tf;
    offset.setOrigin(tf2::Vector3(0, -0.2, 0));
    tf2::Transform sl_off_camera_tf = camera_tf.inverse() * (sl_tf * offset);
    offset.setOrigin(tf2::Vector3(0, 0, 0.2));
    tf2::Transform fa_off_camera_tf = camera_tf.inverse() * (fa_tf * offset);
    offset.setOrigin(tf2::Vector3(0, 0, 0.05));
    tf2::Transform fa_camera_tf = camera_tf.inverse() * (fa_tf * offset);
    tf2::Transform w1_camera_tf = camera_tf.inverse() * w1_tf;
    offset.setOrigin(tf2::Vector3(0, 0, -0.15));
    tf2::Transform w1_off_camera_tf = camera_tf.inverse() * (w1_tf * offset);
    tf2::Transform w2_camera_tf = camera_tf.inverse() * w2_tf;
    offset.setOrigin(tf2::Vector3(0, 0.3, 0));
    tf2::Transform w3_camera_tf = camera_tf.inverse() * (w2_tf * offset);

    tf2::Vector3 world_p = world_camera_tf.getOrigin();
    tf2::Vector3 sl_p = sl_camera_tf.getOrigin();
    tf2::Vector3 sl_off_p = sl_off_camera_tf.getOrigin();
    tf2::Vector3 fa_off_p = fa_off_camera_tf.getOrigin();
    tf2::Vector3 fa_p = fa_camera_tf.getOrigin();
    tf2::Vector3 w1_off_p = w1_off_camera_tf.getOrigin();
    tf2::Vector3 w1_p = w1_camera_tf.getOrigin();
    tf2::Vector3 w2_p = w2_camera_tf.getOrigin();
    tf2::Vector3 w3_p = w3_camera_tf.getOrigin();

    double min_dist = 0.05;
    // std::vector<tf2::Vector3> primary_points {world_p, sl_p, sl_off_p, fa_off_p, fa_p, w1_off_p, w1_p, w2_p, w3_p};
    std::vector<tf2::Vector3> primary_points {fa_p, w1_off_p, w1_p, w2_p, w3_p};

    std::vector<tf2::Vector3>::iterator it;

    std::vector<tf2::Vector3> total_points;

    for (it = primary_points.begin(); it < --primary_points.end(); it++)
    {   
        total_points.push_back(*it);
        tf2::Vector3 diff_vec = *next(it) - *it;
        double dist = (*it).distance(*next(it));
        
        int num_secondary_points = floor(dist / min_dist);
        tf2::Vector3 increment = diff_vec / num_secondary_points;

        for (int i = 0; i < num_secondary_points; i++)
        {
            tf2::Vector3 new_sec_point = (*it) + i * increment;
            total_points.push_back(new_sec_point);
        }
    }

    if (robot_models)
    {
        for (int i = 0; i < total_points.size(); i++)
        {
            viewer_viz->addSphere(pcl::PointXYZ(total_points[i].x(), total_points[i].y(), 
                total_points[i].z()), 0.05, std::to_string(i));
        }
    }
    

    // Data containers used
    CloudPtr cloud_out (new Cloud);
    CloudPtr cloud_viz (new Cloud);
    CloudNormalPtr cloud_with_normals (new CloudNormal);
    IdxClustersPtr clusters (new IdxClusters), small_clusters (new IdxClusters), large_clusters (new IdxClusters);
    pcl::search::KdTree<Point>::Ptr search_tree (new pcl::search::KdTree<Point>);
    
    // Downsample the cloud using a Voxel Grid class
    pcl::VoxelGrid<Point> vg;
    vg.setInputCloud (cloud_global);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.setDownsampleAllData (true);
    vg.filter (*cloud_out);
    pcl::copyPointCloud(*cloud_out, *cloud_viz);

    // Filter point cloud removing everything that is not necessary to obstacle detection
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<Point> extract;
    for (int i = 0; i < cloud_out->points.size(); i++)
    {
        Point pcl_point = cloud_out->points[i];
        tf2::Vector3 point(pcl_point.x, pcl_point.y, pcl_point.z);
        bool robot = false;
        double min_dist = INT16_MAX;
        for (tf2::Vector3 robot_point : total_points)
        {
            double distance = point.distance(robot_point);
            if(distance < min_dist_robot)
            {
                robot = true;
                break;
            }
            if (distance < min_dist)
            {
                min_dist = distance;
            }
        }
        if (robot)
        {
            // Points belonging to the robot
            cloud_viz->points[i].r = 0;
            cloud_viz->points[i].g = 255;
            cloud_viz->points[i].b = 0;
            inliers->indices.push_back(i);
        }
        else if (min_dist < min_dist_interest)
        {
            // Points belonging to the region of interest
            cloud_viz->points[i].r = 255;
            cloud_viz->points[i].g = 0;
            cloud_viz->points[i].b = 0;
        }
        else
        {
            // Points belonging elsewhere
            cloud_viz->points[i].r = 0;
            cloud_viz->points[i].g = 0;
            cloud_viz->points[i].b = 255;
            inliers->indices.push_back(i);
        }
    }

    // With the inliers previusly created segment point cloud
    extract.setInputCloud(cloud_out);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_out);

    // Set up a Normal Estimation class and merge data in cloud_with_normals
    pcl::copyPointCloud (*cloud_out, *cloud_with_normals);
    pcl::NormalEstimation<Point, PointNormal> ne;
    ne.setInputCloud (cloud_out);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*cloud_with_normals);

    // Set up a Conditional Euclidean Clustering class
    pcl::ConditionalEuclideanClustering<PointNormal> cec (true);
    cec.setInputCloud (cloud_with_normals);
    cec.setConditionFunction (&customRegionGrowing);
    cec.setClusterTolerance (cluster_tolerance);
    cec.setMinClusterSize (min_cluster_size);
    cec.setMaxClusterSize (max_cluster_size);
    cec.segment (*clusters);
    cec.getRemovedClusters (small_clusters, large_clusters);

    // Identify clusters from CEC
    // paintFullCluster(-1, cloud_out, small_clusters, 0, 100, 0);
    // paintFullCluster(-1, cloud_out, large_clusters, 0, 0, 100);
    // paintFullCluster(-1, cloud_out, clusters, 255, 255, 0);
    paintRandomCluster(cloud_out, clusters);

    // Obstacles arrays
    std::vector<geometry_msgs::Point> obstacles_centers;
    std::vector<double> obstacles_radiuses;

    // TODO: Admit other types of obstacles other than spheres (real scenario)'
    std::cout << clusters->size() << std::endl;

    if (random_obstacles)
    {

        Point eff;
        eff.x = w3_p.x();
        eff.y = w3_p.y();
        eff.z = w3_p.z();
        viewer_viz->addSphere(eff, 0.02, 
                                      255, 255, 0, "eef");


        // Compute cluster centroid and possibly bounding box
        for (int i = 0; i < clusters->size(); i++)
        {
            CloudPtr obstacle_cloud (new Cloud);
            pcl::ExtractIndices<Point> filter;
            filter.setInputCloud(cloud_out);
            pcl::PointIndices::Ptr filter_indices (new pcl::PointIndices);
            filter_indices->indices = clusters->at(i).indices;
            filter.setIndices(filter_indices);
            filter.filter(*obstacle_cloud);

            // Obstacle message
            geometry_msgs::Point center;

            // // Compute Centroid
            // Eigen::Vector4f obstacle_center;
            // pcl::compute3DCentroid(*obstacle, obstacle_centroid);

            // // Add obstacle to obstacle array msgs
            // center.x = obstacle_center[0];
            // center.y = obstacle_center[1];
            // center.z = obstacle_center[2];
            // obstacles_centers.push_back(center);
            // obstacles_radiuses.push_back(0.05);

            // Compute closest point of cluster to EEF
            float min_dist = MAXFLOAT;
            int min_dist_idx = 0;

            // Compute closest point to EFF
            for (int i = 0; i < obstacle_cloud->points.size(); i++)
            {
                float dist = pcl::euclideanDistance(eff, obstacle_cloud->points[i]);
                if (dist < min_dist)
                {   
                    min_dist = dist;
                    min_dist_idx = i;
                }
            }

            center.x = obstacle_cloud->points[min_dist_idx].x;
            center.y = obstacle_cloud->points[min_dist_idx].y;
            center.z = obstacle_cloud->points[min_dist_idx].z;
            obstacles_centers.push_back(center);
            obstacles_radiuses.push_back(0.05);

            // Draw Spheres on Obstacle
            if (obstacle_models)
            {
                viewer_viz->addSphere(pcl::PointXYZ(center.x, center.y, 
                                      center.z), 0.02, 
                                      255, 0, 0, "sphere" + std::to_string(i));
            }
        }
    }

    // Treat every cluster as a sphere
    if (sphere_obstacles)
    {
        for (int i = 0; i < clusters->size(); i++)
        {
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

            // Add obstacle to obstacle array msgs
            geometry_msgs::Point center;
            center.x = model_coefficients[0];
            center.y = model_coefficients[1];
            center.z = model_coefficients[2];
            obstacles_centers.push_back(center);
            obstacles_radiuses.push_back(model_coefficients[3]);

            if (obstacle_models)
            {
                viewer_viz->addSphere(pcl::PointXYZ(center.x, center.y, center.z), model_coefficients[3], 
                                      255, 0, 0, "sphere" + std::to_string(i));
            }
        
            std::cout << "Cluster " << i << "\n";
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
    if (viz_cloud)
    {
        viewer_viz->updatePointCloud(cloud_viz, "viz_cloud");
    }
    else
    {
        viewer_viz->updatePointCloud(cloud_out, "viz_cloud");
    }
    viewer_viz->spinOnce();
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "obstacle_detection");
    ros::NodeHandle nh;

    // Random seed generator
    srand(time(NULL));

    // Dynamic reconfigure init and callback
    dynamic_reconfigure::Server<iris_cobot::ObstacleDetectionConfig> server;
    dynamic_reconfigure::Server<iris_cobot::ObstacleDetectionConfig>::CallbackType cobotConfigCallback;
    cobotConfigCallback = boost::bind(&parameterConfigure, _1, _2);
    server.setCallback(cobotConfigCallback);

    // Transform listener
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Obtain camera fixed transform
    geometry_msgs::TransformStamped camera_tf_msg;
    while (nh.ok())
    {
        try
        {
            camera_tf_msg = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(0.1).sleep();
            continue;
        }
    }
    tf2::fromMsg(camera_tf_msg, camera_tf);

    // Obstacles Publisher
    obstacles_pub = nh.advertise<iris_cobot::Obstacles>("obstacles", 1);

    // Setup Viewers
    viewer_viz = normalVis(cloud_global, "3DVizViewer", "viz_cloud");

    // Camera Subscription service
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_callback);

    // Spin
    ros::spin();
}