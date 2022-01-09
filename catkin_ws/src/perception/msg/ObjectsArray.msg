# WATanoBus - Perception and Prediction: Version: V1
# Prepared by: Neel Bhatt
# Last Modified by: _

#-------------------- Perception and Prediction: Object Array ROS MSG FORMAT --------------------#
# Listens to ROS node: /pylon_camera_node_left /pylon_camera_node_center /pylon_camera_node_right /pylon_camera_node_rear /velodyne_points /applanix/gps_odom /tf
# Published ROS node: /perception_prediction/detection_array
#----------------------------------------------------------------------------------------------#

std_msgs/Header header   # UTC Time From Applanix $GPRMC
ObjectInstance[] objects # Array of detected objects
