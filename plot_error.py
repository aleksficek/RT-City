#!/usr/bin/env python
import rospy
import numpy
import message_filters as mf
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from autoware_msgs.msg import DetectedObjectArray

def callback_error(gps_msg, bbox_msg):
    point = Point()

    point.x = abs(gps_msg.pose.pose.position.x - bbox_msg.objects[0].pose.position.x)
    point.y = abs(gps_msg.pose.pose.position.y - bbox_msg.objects[0].pose.position.y)
    point.z = abs(gps_msg.pose.pose.position.z - bbox_msg.objects[0].pose.position.z)

    error_pub.publish(point)


if __name__ == '__main__':
    global error_pub
    rospy.init_node('plot_position_error', anonymous=True)

    gps_sub = mf.Subscriber("/ego_odom", Odometry)
    bbox_sub = mf.Subscriber("node1/bounding_boxes", DetectedObjectArray)

    error_pub = rospy.Publisher("/bus_vs_node_pos_error", Point,queue_size=1)

    ts = mf.TimeSynchronizer([gps_sub, bbox_sub], 10)

    ts.registerCallback(callback_error)
    
    rospy.spin()