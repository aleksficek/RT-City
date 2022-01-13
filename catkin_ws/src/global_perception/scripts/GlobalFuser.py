#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
import message_filters as mf
# from GlobalBBox import GlobalBBoxBuffer
from global_perception.msg import GlobalBBoxBuffer

from global_perception.msg import ChildGlobalBBoxBuffer
from BBoxPredictor import BBoxPredictor

NODE_RATE = 20 # Hz

class GlobalFuser():
    def __init__(self):
        rospy.init_node('global_perception', anonymous=False)
        self.rate = rospy.Rate(NODE_RATE)

        self.sub_n1 = mf.Subscriber('node1/bounding_boxes', BoundingBoxArray)
        self.sub_n2 = mf.Subscriber('node2/bounding_boxes', BoundingBoxArray)
        self.sub_n3 = mf.Subscriber('node3/bounding_boxes', BoundingBoxArray)
        self.time_synch = mf.TimeSynchronizer([self.sub_n1, self.sub_n2, self.sub_n3], 10)
        self.time_synch.registerCallback(self.nodeCallback)
        
        self.master_buffer = GlobalBBoxBuffer()

        self.pub = rospy.Publisher('global_fused_bboxes', GlobalBBoxBuffer, queue_size=10)
        self.predictions = BBoxPredictor() # class for holding predictions for ID'd bbox
        
        # Run the fuser at 20Hz
        while not rospy.is_shutdown():
            self.fuserRun50ms()
            self.rate.sleep()

    def fuserRun50ms(self, event=None):
        # process bboxes for this time step
        self.pub.publish(self.master_buffer)
        self.buffer = GlobalBBoxBuffer()

    def nodeCallback(self, data_n1, data_n2, data_n3):
        # create new bbox to put in global buffer
        self.master_buffer.node1_buffer.append(data_n1)
        self.master_buffer.node2_buffer.append(data_n2)
        self.master_buffer.node3_buffer.append(data_n3)
        # self.fuserRun50ms(data_n1, data_n2, data_n3)
        

if __name__ == '__main__':
    try:
        GlobalFuser()
    except rospy.ROSInterruptException:
        pass
