#''' 
#    Purpose: Classification and BBox Regression - ROSified
#    Note: Topic names can be modified in yolo_console file
#    Subscribed topics: /pylon_camera_node_center/image_rect
#    Published topic: /bbox_array
#
#    Project: WATonoBus
#    Author: Neel Bhatt    
#
#    DO NOT share without permission from the author
#'''

cmake_minimum_required(VERSION 2.8.3)
project(perception_prediction)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    ObjectInstance.msg
    ObjectsArray.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
)
