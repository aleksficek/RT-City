#!/usr/bin/env python
''' 
    Purpose: Filter ground points from raw transformed/ground aligned lidar points
    Subscribed topics: /rslidar_points_front/ground_aligned
    Pulished topic: /rslidar_points_front/ground_filtered

    Project: WATonoBus
    Author: Neel Bhatt
    Date: Feb 26, 2022
    Do not share, copy, or use without seeking permission from the author
'''

# Run with Python2
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pcl
import numpy as np
import cv2
import csv
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from visualization_msgs.msg import Marker, MarkerArray
import time
import getpass

np.set_printoptions(suppress=True)

# pointcloud processing
def callback_pointcloud(raw_cloud):
    global bounds_array
    # print("----- LIDAR: " + str(time.time()) + " -----")
    start = time.time()

    # Convert pointcloud2 msg to np record array (with x,y,z,intensity, and rings fields)
    points_np_record = ros_numpy.point_cloud2.pointcloud2_to_array(raw_cloud) # np.array(...) allows for write access to points_np_record
    points_np_record = points_np_record.copy()

    # Convert np record array to np array (with just x,y,z)
    points_np = np.zeros((points_np_record['x'].flatten().shape[0],3))
    points_np[:,0]=points_np_record['x'].flatten()
    points_np[:,1]=points_np_record['y'].flatten()
    points_np[:,2]=points_np_record['z'].flatten()
    points_np_intensity=points_np_record['intensity'].flatten()
    # points_np_ring=points_np_record['ring'].flatten()
    

    # Transformation from LIDAR to Image Plane
    lidar_points = np.insert(points_np,3,[1.0],axis = 1)
    z_axis_filtered_indices = np.logical_and(lidar_points[:,2] > -9.20,lidar_points[:,2] < -1)
    x_axis_filtered_indices = np.logical_and(lidar_points[:,0] > -13.0,lidar_points[:,0] < 13.0)
    valid = np.logical_and(z_axis_filtered_indices,x_axis_filtered_indices)
    lidar_points = lidar_points[valid]
    points_np_intensity=points_np_intensity[valid]
    # points_np_ring=points_np_ring[valid]

    for_ground = points_np_record.copy()

    for_ground = np.resize(for_ground,lidar_points[:,0].shape)
    # print("After:",points_np_record.shape)
    for_ground['x']=lidar_points[:,0]
    for_ground['y']=lidar_points[:,1]
    for_ground['z']=lidar_points[:,2]
    for_ground['intensity']=points_np_intensity
    # for_ground['ring']=points_np_ring

    # Convert np record array to Pointcloud2 msg
    ground_filtered_cloud = ros_numpy.point_cloud2.array_to_pointcloud2(for_ground, stamp = raw_cloud.header.stamp, frame_id='ground_aligned')
    pub_ground_filtered_cloud.publish(ground_filtered_cloud)

    print("------------ End ------------")
    bounds_array = np.array([])
    print("Took "+str((time.time()-start)*1000)+" ms")

def filter_ground():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'filter_ground' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ground_filtering', anonymous=True)

    rospy.Subscriber("/rslidar_points_front/ground_aligned", PointCloud2, callback_pointcloud)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    # Real-time operation
    pub_ground_filtered_cloud = rospy.Publisher('/rslidar_points_front/ground_filtered', PointCloud2, queue_size=1)

    filter_ground()