/*
''' 
    Purpose: Transform raw lidar points in sensor node lidar frame to ground alinged frame
    Subscribed topics: /rslidar_points_front
    Pulished topic: /rslidar_points_front/ground_aligned

    Project: WATonoBus
    Author: Neel Bhatt
    Date: Feb 26, 2022
    Do not share, copy, or use without seeking permission from the author
'''   
*/

#include "ros/ros.h"
#include <iostream>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <transform_publisher_infra/calib_robosenseConfig.h>

// RPY
float roll_alpha=0.85*M_PI/180;
float pitch_beta=-55*M_PI/180;
float yaw_gamma=0*M_PI/180;

// Translations
float translation_x=0;
float translation_y=0;
float translation_z=0;


void callback_dynamic_reconfig(transform_publisher::calib_robosenseConfig &config, uint32_t level) {
    roll_alpha = config.roll_alpha*M_PI/180;
    pitch_beta = config.pitch_beta*M_PI/180;
    yaw_gamma = config.yaw_gamma*M_PI/180;

    translation_x = config.translation_x;
    translation_y = config.translation_y;
    translation_z = config.translation_z;
}

Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

// Compute Transformation Matrix given RPY angles about fixed axis as well as translation vector
void transformation_matrix(float roll_alpha,float pitch_beta,float yaw_gamma,float translation_x,float translation_y,float translation_z){
    Eigen::Matrix3f rotation_matrix;
    // Construct Rotation Matrix
    rotation_matrix(0,0)=cos(yaw_gamma)*cos(pitch_beta);
    rotation_matrix(0,1)=-sin(yaw_gamma)*cos(roll_alpha)+cos(yaw_gamma)*sin(pitch_beta)*sin(roll_alpha);
    rotation_matrix(0,2)=sin(yaw_gamma)*sin(roll_alpha)+cos(yaw_gamma)*sin(pitch_beta)*cos(roll_alpha);
    rotation_matrix(1,0)=sin(yaw_gamma)*cos(pitch_beta);
    rotation_matrix(1,1)=cos(yaw_gamma)*cos(roll_alpha)+sin(yaw_gamma)*sin(pitch_beta)*sin(roll_alpha);
    rotation_matrix(1,2)=-cos(yaw_gamma)*sin(roll_alpha)+sin(yaw_gamma)*sin(pitch_beta)*cos(roll_alpha);
    rotation_matrix(2,0)=-sin(pitch_beta);
    rotation_matrix(2,1)=cos(pitch_beta)*sin(roll_alpha);
    rotation_matrix(2,2)=cos(pitch_beta)*cos(roll_alpha);
    transform_1.block<3,3>(0,0)=rotation_matrix;
    // Construct Translation Matrix
    transform_1(0,3) = translation_x;
    transform_1(1,3) = translation_y;
    transform_1(2,3) = translation_z;
    transform_1 = transform_1.inverse().eval();
}

sensor_msgs::PointCloud2 transformed_cloud_msg;
ros::Publisher pub_transformed_cloud;

// Transformer
void callback_pointcloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& raw_cloud){
    transformation_matrix(roll_alpha,pitch_beta,yaw_gamma,translation_x,translation_y,translation_z);


    std::cout << "Roll:" << roll_alpha << "  Pitch:" << pitch_beta << "  Yaw:" << yaw_gamma << std::endl;
    std::cout << "X:" << translation_x << "  Y:" << translation_y << "  Z:" << translation_z << std::endl;

    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    
    // Convert from PC2 ROS msg to PCL (XYZI) object
    pcl::fromROSMsg(*raw_cloud, pcl_cloud);

    // Transformation per point
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::transformPointCloud(pcl_cloud, *transformed_cloud, transform_1);
    
    // Convert from PCL (XYZI) object to PC2 ROS msg
    pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
    
    // Publish transformed cloud msg    
    transformed_cloud_msg.header.stamp = raw_cloud->header.stamp;
    transformed_cloud_msg.header.frame_id = "ground_aligned";
    pub_transformed_cloud.publish(transformed_cloud_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rslidar_F_transform_node");

    ros::NodeHandle nh;

    // For Calibration
    // dynamic_reconfigure::Server<transform_publisher::calib_robosenseConfig> server;
    // dynamic_reconfigure::Server<transform_publisher::calib_robosenseConfig>::CallbackType f;

    // f = boost::bind(&callback_dynamic_reconfig, _1, _2);
    // server.setCallback(f);

    ros::Subscriber sub = nh.subscribe("/rslidar_points_front", 1, callback_pointcloud);
    pub_transformed_cloud = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_points_front/ground_aligned", 1);
    ros::spin();

  return 0;
}