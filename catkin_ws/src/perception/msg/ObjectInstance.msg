# WATanoBus - Perception and Prediction: Version: V1
# Prepared by: Neel Bhatt
# Last Modified by: _

#-------------------- Perception and Prediction: Object ROS MSG FORMAT --------------------#
# Listens to ROS node: none
# Published ROS node: none
#----------------------------------------------------------------------------------#

std_msgs/Header                 header	     # UTC time from Applanix $GPRMC

uint32                          id           # From tracker
string                          label        # Class list TBD
float32                         confidence   #Score as defined by the detection, Optional

#---------- 3D BB (LIDAR+Images) ----------#
string                          ptcloud_frame     #3D sensor frame with respect to which the object location is specified
geometry_msgs/Point             position     # X,Y,Z in meters
geometry_msgs/Twist             velocity     # meters/second
geometry_msgs/Twist             acceleration # meters/second^2
geometry_msgs/Vector3           box_dimensions #meters

sensor_msgs/PointCloud2         fusion_filtered_Pointcloud

bool                            pose_reliable
bool                            velocity_reliable
bool                            acceleration_reliable

#---------- 2D BB (All Cameras) ----------#
string                          image_frame # Image coordinate Frame (will be center camera)
int32                           x           # X coord in image space(pixel) of the center point of the BB
int32                           y           # Y coord in image space(pixel) of the center point of the BB
int32                           width       # Width of the BB in pixels
int32                           height      # Height of the BB in pixels

sensor_msgs/Image               roi_cropped_image


#---------- Prediction ----------#
uint32				            horizon_pred
geometry_msgs/Point[]           candidate_prediction_trajectories # consists of a set of X,Y,Z measures in meters

#------------ Info --------------#
string[]                        additional_user_defined_info
