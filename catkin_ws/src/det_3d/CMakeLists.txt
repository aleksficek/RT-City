cmake_minimum_required(VERSION 3.0.2)
project(det_3d)

find_package(catkin REQUIRED COMPONENTS
    autoware_msgs
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
    pcl_ros
    cv_bridge
    image_transport
    message_generation
    tf2
    tf2_ros
    tf2_msgs
    tf2_sensor_msgs
    tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                 pcl_conversions
                 pcl_ros
                 message_runtime

#  DEPENDS other non-ROS libs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/aaroscpp_pcl_example.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
