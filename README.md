# RT-City

Welcome to the RT-City Project. This repo encapsulates all things for the project, please refer to the corresponding folder for information and code on sub fields of the project.

## General Setup

1. Download (AnyDesk)[https://anydesk.en.uptodown.com/windows/versions], any 5.x.x version will not have ads and are recommended. 

2. Join remote computer using AnyDesk, ID and password are in RT-City-FYDP teams chat.

## Launching The Stack (In separate terminals)
1. Roscore
2. rosparam set use_sim_time true && rosbag play BAG --clock -l
3. python rotate_and_pub_right_image_compressed.py
4. roslaunch transform_publisher_infra transform_pub.launch
5. rosrun transform_publisher_infra transform_points_infra
6. LD_LIBRARY_PATH=./:$LD_LIBRARY_PATH ./uselib_left data/coco.names cfg/yolov4.cfg yolov4.weights basler_camera --i 0
7. LD_LIBRARY_PATH=./:$LD_LIBRARY_PATH ./uselib_right data/coco.names cfg/yolov4.cfg yolov4.weights basler_camera --i 0
python2 ground_filter.py 
8. rosrun det_3d detection_node
9. roslaunch range_vision_fusion range_vision_fusion.launch
10. rosrun global_perception GlobalFuser.py
11. rviz (optional) 
12. rqt (optional)