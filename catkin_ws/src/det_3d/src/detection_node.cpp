// // C++ imports *****************************************************************
#include <limits.h>
#include <math.h>
#include <sstream>
#include <vector>

// // ROS/PCL imports *************************************************************
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/PointIndices.h"
#include "pcl/common/angles.h"
#include "pcl/common/common.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
//
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/TransformStamped.h>
#include "shape_msgs/SolidPrimitive.h"
#include "visualization_msgs/Marker.h"

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include "det_3d/detection_node.h"

ros::Publisher pub_autoware_objects;
ros::Publisher pub_jsk_bboxes;
ros::Publisher pub_above_ground;
ros::Publisher pub_voxelized;

/*
steps:
    1. ingest ROS pointcloud2 (PointClou2) and convert to PCL model_types
    2. rotate PointCloud2 to align LiDAR ground plane with real ground plane
    3. filter out ground with PCL sac_segmentation
    4. Downsample remaining pointcloud (voxelize)
    5. Run clustering on remaning pointcloud
    6. Iterate over all clusters found in pointcloud:
       for each cluster in all_clusters:
            -   fit bounding box DetectedObject for cluster
            -   DetectedObjectArray.push_back(DetectedObject for cluster)
       end
    7. publish DetectedObjectArray
*/

Box bbox_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& c, const int id){
    // Compute the bounding box height (to be used later for recreating the box)
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*c, min_pt, max_pt);
    const float box_height = max_pt.z - min_pt.z;
    const float box_z = (max_pt.z + min_pt.z)/2;

    // Compute the cluster centroid
    Eigen::Vector4f pca_centroid;
    pcl::compute3DCentroid(*c, pca_centroid);

    // Squash the cluster to x-y plane with z = centroid z
    for (size_t i = 0; i < c->size(); ++i){
        c->points[i].z = pca_centroid(2);
    }

    // Compute principal directions & Transform the original cloud to PCA coordinates
    pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(c);
    pca.project(*c, *pca_projected_cloud);

    const auto eigen_vectors = pca.getEigenVectors();

    // Get the minimum and maximum points of the transformed cloud.
    pcl::getMinMax3D(*pca_projected_cloud, min_pt, max_pt);
    const Eigen::Vector3f meanDiagonal = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf quaternion(eigen_vectors); // Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f position = eigen_vectors * meanDiagonal + pca_centroid.head<3>();
    const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), box_height);

    return Box(id, position, dimension, quaternion);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // std::cout << "Cloud size original" << cloud_msg->height<<std::endl;
    // std::cout << "Cloud size original" << cloud_msg->width<<std::endl;
    size_t original_num_pts = 0, ground_filtered_num_pts = 0, voxelized_num_pts = 0;
    const int ransac_iter = 150;
    const float threshold = 0.03;

    std::cout << "input pc size: " << cloud_msg->height * cloud_msg->width << std::endl;
    // 1. type conversion
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*cloud_msg, cloud);

    // 3. ground plane filtering

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_iter); // setting max iterations of RANSAC to 30
    seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
    seg.setDistanceThreshold(0.2); // setting ground threshold

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud.makeShared());
    // std::cout << "Cloud size" << cloud.size()<<std::endl;
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty())
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int idx : inliers->indices){
        ground_cloud->points.push_back(cloud.points[idx]);
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);
    pub_above_ground.publish(obstacle_cloud);
    ground_filtered_num_pts = obstacle_cloud->size();
    std::cout << "ground filtered pc size: " << ground_filtered_num_pts << std::endl;

    // 4. voxel downsampling

    const float filter_res = 0.08;
    pcl::PointCloud<pcl::PointXYZ> vox_cloud;

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(obstacle_cloud);
    vg.setLeafSize(filter_res, filter_res, filter_res);
    vg.filter(vox_cloud);

    pub_voxelized.publish(vox_cloud);

    // std::cout << "Cloud size" << vox_cloud.size()<<std::endl;
    pcl::PointCloud<pcl::PointXYZ> vox_cloud_filtered;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(vox_cloud, vox_cloud_filtered, indices);
    std::cout << "voxelized pc size: " << vox_cloud_filtered.points.size () << std::endl;


    // 5. clustering

    const float cluster_tolerance = 4.;
    const int min_size = 25;
    const int max_size = 5000;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_xyz;

    // euclidean clustering to group detected obstacles

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);


    tree->setInputCloud(vox_cloud_filtered.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(vox_cloud_filtered.makeShared());
    ec.extract(cluster_indices);

    if(cluster_indices.empty()){
        std::cout << "CLUSTERING NOT HAPPENING" << std::endl;
    }

    std::cout << "CLUSTER INDICES SIZE (i.e. number of objects): " << cluster_indices.size() << std::endl;

    for (auto& getIndices : cluster_indices)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (auto& index : getIndices.indices){
            cluster->points.push_back(vox_cloud.points[index]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters_xyz.push_back(cluster);
    }

    // converting clusters to autoware messages

    std::size_t obstacle_id_ = (obstacle_id_ < SIZE_MAX)? ++obstacle_id_ : 0;

    autoware_msgs::DetectedObjectArray autoware_objects;
    autoware_objects.header.stamp = ros::Time::now();
    autoware_objects.header.frame_id = "ground_aligned";

    jsk_recognition_msgs::BoundingBoxArray jsk_bboxes;
    jsk_bboxes.header.stamp = ros::Time::now();
    jsk_bboxes.header.frame_id = "ground_aligned";

    for (auto& cluster : clusters_xyz){
        Box temp = bbox_compute(cluster, obstacle_id_);

        // autoware msg
        autoware_msgs::DetectedObject autoware_object; // if this gets manually deleted, will the ref for this bbox in the bbox_array also drop?

        autoware_object.id = temp.id;
        autoware_object.label = "unknown";
        autoware_object.header.stamp = ros::Time::now();
        autoware_object.header.frame_id = "ground_aligned";
        autoware_object.pose_reliable = true;
        autoware_object.valid = true;
        // autoware_object.score = 1.0f; // TODOt
        // autoware_object.pose = pose_transformed; TODO

        autoware_object.dimensions.x = temp.dimension.x();
        autoware_object.dimensions.y = temp.dimension.y();
        autoware_object.dimensions.z = temp.dimension.z();

        // autoware_object.pose.position = temp.position;
        autoware_object.pose.position.x = temp.position.x();
        autoware_object.pose.position.y = temp.position.y();
        autoware_object.pose.position.z = temp.position.z();

        // autoware_object.pose.orientation = temp.quaternion;
        autoware_object.pose.orientation.w = temp.quaternion.w();
        autoware_object.pose.orientation.x = temp.quaternion.x();
        autoware_object.pose.orientation.y = temp.quaternion.y();
        autoware_object.pose.orientation.z = temp.quaternion.z();

        autoware_objects.objects.emplace_back(autoware_object);

        // jsk msg
        jsk_recognition_msgs::BoundingBox jsk_bbox;
        // jsk_bbox.header = header;
        // jsk_bbox.pose = pose_transformed;
        jsk_bbox.header.stamp = ros::Time::now();
        jsk_bbox.header.frame_id = "ground_aligned";
        jsk_bbox.label = 123; // this is random, look into this
        // jsk_bbox.value = 1.0f;

        jsk_bbox.dimensions.x = temp.dimension.x();
        jsk_bbox.dimensions.y = temp.dimension.y();
        jsk_bbox.dimensions.z = temp.dimension.z();

        // jsk_bbox.pose.position = temp.position;
        jsk_bbox.pose.position.x = temp.position.x();
        jsk_bbox.pose.position.y = temp.position.y();
        jsk_bbox.pose.position.z = temp.position.z();

        // jsk_bbox.pose.orientation = temp.quaternion;
        jsk_bbox.pose.orientation.w = temp.quaternion.w();
        jsk_bbox.pose.orientation.x = temp.quaternion.x();
        jsk_bbox.pose.orientation.y = temp.quaternion.y();
        jsk_bbox.pose.orientation.z = temp.quaternion.z();

        jsk_bboxes.boxes.emplace_back(jsk_bbox);
    }

    pub_autoware_objects.publish(autoware_objects);
    pub_jsk_bboxes.publish(jsk_bboxes);
    // end of clustering, all object clusters stored in clusters array
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "lidar_detection");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/rslidar_points_front/ground_aligned", 1, cloud_cb);
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("/autoware_bboxes", 1);
  pub_autoware_objects = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_objects", 1);
  pub_jsk_bboxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bboxes_jsk_format", 1);
  pub_above_ground = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/above_ground", 1);
  pub_voxelized = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/after_voxelization", 1);
  ros::spin();
}
