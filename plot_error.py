#!/usr/bin/env python
import rospy
import numpy
import tf
import message_filters as mf
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from autoware_msgs.msg import DetectedObjectArray

def callback_error(gps_msg, bbox_msg):
    point = Point()
    tf_listener.getLatestCommonTime("/ground_aligned", "/map")

    bbox_pose_map = tf_listener.transformPose("/map", bbox_msg.objects[0].pose)
    point.x = abs(gps_msg.pose.pose.position.x - bbox_pose_map.position.x)
    point.y = abs(gps_msg.pose.pose.position.y - bbox_pose_map.position.y)
    point.z = abs(gps_msg.pose.pose.position.z - bbox_pose_map.position.z)

    error_pub.publish(point)


if __name__ == '__main__':
    global error_pub, tf_listener
    rospy.init_node('plot_position_error', anonymous=True)

    tf_listener = tf.TransformListener()

    gps_sub = mf.Subscriber("/ego_odom", Odometry)
    bbox_sub = mf.Subscriber("/global_fused_bboxes", DetectedObjectArray)

    error_pub = rospy.Publisher("/bus_vs_node_pos_error", Point,queue_size=1)

    ts = mf.ApproximateTimeSynchronizer([gps_sub, bbox_sub], 10, 0.5)

    ts.registerCallback(callback_error)
    
    rospy.spin()