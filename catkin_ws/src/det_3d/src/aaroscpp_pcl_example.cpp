// // C++ imports *****************************************************************
#include <limits.h>
#include <math.h>
#include <sstream>
#include <vector>
//
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
//

#include "det_3d/aaroscpp_pcl_example.h"

ros::Publisher pub_autoware_objects;
ros::Publisher pub_above_ground;


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

    // 1. type conversion
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // // 2. rotation
    // // not yet implemented, talk to Neel about this - TODO
    // // 3. ground plane filtering

    pcl::SACSegmentation<pcl::PCLPointCloud2> seg;
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(30); // setting max iterations of RANSAC to 30
    seg.setDistanceThreshold(0.3); // setting ground threshold

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloudPtr);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty())
    {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // // now removing the ground plane from pointcloud
    //
    pcl::PCLPointCloud2Ptr obstacle_cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2Ptr ground_cloud(new pcl::PCLPointCloud2);

    // pcl::PointCloud<pcl::PointXYdZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //
    for (int idx : inliers->indices){
        ground_cloud->data.push_back(cloudPtr->data[idx]);
    }
    //
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(cloudPtr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);
    // // end of ground plane filtering, obstacle_cloud is Ptr for next section, contains above ground plane points only
    pcl::PointCloud<pcl::PointXYZ> above_ground_xyz;
    pcl::fromPCLPointCloud2(*obstacle_cloud, above_ground_xyz);
    sensor_msgs::PointCloud2 above_ground_pc;
    pcl::toROSMsg(above_ground_xyz, above_ground_pc);
    pub_above_ground.publish(above_ground_pc);
    // // 4. voxel downsampling




    float filter_res = 0.02; // can change this after
    pcl::PCLPointCloud2Ptr voxelized_ptr(new pcl::PCLPointCloud2);

    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud(obstacle_cloud);
    vg.setLeafSize(filter_res, filter_res, filter_res);
    vg.filter(*voxelized_ptr);

    // end of voxel downsampling, voxelized_ptr is Ptr for next section.

    // 5. clustering

    const float cluster_tolerance = 0.6;
    const int min_size = 5000;
    const int max_size = 10;
    //
    std::vector<pcl::PCLPointCloud2Ptr> clusters_pc2;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_xyz;

    // // euclidean clustering to group detected obstacles

    // convert to xyz to feed into kdtree

    pcl::PointCloud<pcl::PointXYZ> *voxelized_xyz = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_xyz_ptr (voxelized_xyz);
    pcl::fromPCLPointCloud2(*voxelized_ptr, *voxelized_xyz_ptr);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(voxelized_xyz_ptr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(voxelized_xyz_ptr);
    ec.extract(cluster_indices);
    //
    // TODO should this processing below be PointXYZ or PCLPointCloud2 ?
    for (auto& getIndices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (auto& index : getIndices.indices){
            cluster->points.push_back(voxelized_xyz_ptr->points[index]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters_xyz.push_back(cluster);
    }
    // converting clusters to autoware messages

    std::size_t obstacle_id_ = (obstacle_id_ < SIZE_MAX)? ++obstacle_id_ : 0;

    autoware_msgs::DetectedObjectArray autoware_objects;
    //
    for (auto& cluster : clusters_xyz){
        Box temp = bbox_compute(cluster, obstacle_id_);

        autoware_msgs::DetectedObject autoware_object; // if this gets manually deleted, will the ref for this bbox in the bbox_array also drop?
        // autoware_object.header = ; // TODO
        autoware_object.id = temp.id;
        autoware_object.label = "unknown";
        // autoware_object.score = 1.0f; // TODOt
        // autoware_object.pose = pose_transformed; TODO
        autoware_object.pose_reliable = true;
        autoware_object.dimensions.x = temp.dimension(0);
        autoware_object.dimensions.y = temp.dimension(1);
        autoware_object.dimensions.z = temp.dimension(2);
        autoware_object.valid = true;
        autoware_objects.objects.emplace_back(autoware_object);
        // TODO walk through ptr/PCL data types again carefully ensure output is correct format
    }

    pub_autoware_objects.publish(autoware_objects);
    // // can also simulateneously publish jsk_recognition_msgs
    // // end of clustering, all object clusters stored in clusters array
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/rslidar_points_front/ground_aligned", 1, cloud_cb);
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("/autoware_bboxes", 1);
  pub_autoware_objects = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_objects", 1);
  pub_above_ground = nh.advertise<sensor_msgs::PointCloud2>("/above_ground", 1);
  ros::spin();
}


/*
TODO
- do we need to change queue sizes if working with big pointclouds with lots of processing going on

- make another publisher that outputs only non-ground points (obstacle_clouds)
- rotate incoming pointcloud (step 2) to get ground planes to align
- somehow do the pose update for the autoware DetectedObject


- - make another publisher that outputs jsk_recognition_msgs for viz

*/


// todo after neel's call
// - sub to his rotated pointcloud and viz with jsk in rviz
// - see how good bbox_compute is
// - if bbox_compute not good, look at L-shape box fitting
// - L shape fitting (ditch PCA)
// - leave queue size to 1
// -
