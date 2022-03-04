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
)

find_package(PCL 1.8 REQUIRED)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8) 
message("PCL_LIBRARIES - ${PCL_LIBRARIES}")
message("CHECKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                 pcl_conversions
                 pcl_ros
                 message_runtime

#  DEPENDS other non-ROS libs
)
include_directories(${PCL_INCLUDE_DIRS})



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/aaroscpp_pcl_example.cpp
)



add_executable(aaroscpp_hello_world src/aaroscpp_hello_world.cpp)
target_link_libraries (aaroscpp_hello_world ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(aaroscpp_pcl_example ${catkin_LIBRARIES})
# add_dependencies(aaroscpp_pcl_example aaroscpp_pcl_example)

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
