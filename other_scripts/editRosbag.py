import rosbag
import rospy
import math
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

write_bag = rosbag.Bag('/media/c66tang/Data/FYDP/bagfiles/EditedBag2.bag', 'w')
read_bag = rosbag.Bag('/media/c66tang/Data/FYDP/bagfiles/2022-02-02-21-14-37.bag')

good_topics = [           
    '/carla/camera/rgb/node1_camera_left/camera_info',              
    '/carla/camera/rgb/node1_camera_left/image_color',                    
    '/carla/camera/rgb/node1_camera_right/camera_info',              
    '/carla/camera/rgb/node1_camera_right/image_color',                  
    '/carla/camera/rgb/node2_camera_left/camera_info',             
    '/carla/camera/rgb/node2_camera_left/image_color',                    
    '/carla/camera/rgb/node2_camera_right/camera_info',                
    '/carla/camera/rgb/node2_camera_right/image_color',                        
    '/carla/hero/gnss/default/fix',
    '/carla/hero/imu',                            
    '/carla/hero/objects',                                  
    '/carla/hero/odometry',                                                
    '/carla/hero/vehicle_info',                              
    '/carla/hero/vehicle_status',                         
    '/carla/lidar/node1_lidar/point_cloud',                       
    '/carla/lidar/node2_lidar/point_cloud',                        
    '/carla/objects',           
    '/carla/vehicle/013/odometry',                                       
    '/carla/vehicle/014/odometry',                                                
    '/clock',                                     
    '/rosout',
    '/rosout_agg',                                                      
    '/tf'                                    
]
id = 1
publish_topic = ""
publish_dict = {}

node_divider_slope = 11./23.
for topic, msg, t in read_bag.read_messages(topics=good_topics):
    if topic == '/carla/objects':
        publish_dict['node1'] =  BoundingBoxArray()
        publish_dict['node2'] =  BoundingBoxArray()

        publish_dict['node1'].header = msg.header
        publish_dict['node1'].header.frame_id = 'map'
        publish_dict['node2'].header = msg.header
        publish_dict['node2'].header.frame_id = 'map'

        for submsg in msg.objects:
            dict_key = ''
            divider_y = (node_divider_slope*submsg.pose.position.x) - 67.91
            if abs(divider_y - submsg.pose.position.y) < (submsg.shape.dimensions[1] + 3.):
                # we are close to end of FOV, break up BBox
                node_bbox = BoundingBox()
                node_bbox.value = id
                id += 1
                node_bbox.header = submsg.header
                node_bbox.header.frame_id = 'map'
                node_bbox.pose = submsg.pose
                node_bbox.dimensions.x = submsg.shape.dimensions[0]/2.
                node_bbox.dimensions.y = submsg.shape.dimensions[1]
                node_bbox.dimensions.z = submsg.shape.dimensions[2]

                node2_bbox = BoundingBox()
                node2_bbox.value = id
                id += 1
                node2_bbox.header = submsg.header
                node2_bbox.header.frame_id = 'map'
                node2_bbox.pose.orientation = submsg.pose.orientation
                node2_bbox.pose.position.x = submsg.pose.position.x - submsg.shape.dimensions[0]*math.cos(45)
                node2_bbox.pose.position.y = submsg.pose.position.y + submsg.shape.dimensions[0]*math.cos(45)
                node2_bbox.pose.position.z = submsg.pose.position.z
                node2_bbox.dimensions.x = submsg.shape.dimensions[0]/2.
                node2_bbox.dimensions.y = submsg.shape.dimensions[1]
                node2_bbox.dimensions.z = submsg.shape.dimensions[2]

                publish_dict['node1'].boxes.append(node_bbox)
                publish_dict['node2'].boxes.append(node2_bbox)
                continue
            if submsg.pose.position.y > divider_y:
                publish_topic = 'node1/bounding_boxes'
                dict_key = 'node1'
            else:
                publish_topic = 'node2/bounding_boxes'
                dict_key = 'node2'

            node_bbox = BoundingBox()
            node_bbox.value = id
            id += 1
            node_bbox.header = submsg.header
            node_bbox.header.frame_id = 'map'
            node_bbox.pose = submsg.pose
            node_bbox.dimensions.x = submsg.shape.dimensions[0]
            node_bbox.dimensions.y = submsg.shape.dimensions[1]
            node_bbox.dimensions.z = submsg.shape.dimensions[2]

            publish_dict[dict_key].boxes.append(node_bbox)

        id = 1
        for key in publish_dict:
            if len(publish_dict[key].boxes) > 0:
                if key == 'node1':
                    write_bag.write('node1/bounding_boxes', publish_dict['node1'])
                elif key == 'node2':
                    write_bag.write('node2/bounding_boxes', publish_dict['node2'])

    write_bag.write(topic, msg)





write_bag.close()
read_bag.close()