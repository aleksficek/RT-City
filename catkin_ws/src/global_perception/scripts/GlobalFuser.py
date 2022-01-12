#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from GlobalBBox import GlobalBBoxBuffer
from BBoxPredictor import BBoxPredictor

NODE_RATE = 20 # Hz

class GlobalFuser():
    def __init__(self):
        rospy.init_node('global_perception', anonymous=False)
        self.rate = rospy.Rate(NODE_RATE)
        self.sub = rospy.Subscriber('chatter', String, self.nodeCallback)
        self.pub = rospy.Publisher('global_fused_bboxes', String, queue_size=10)
        # self.rawBuffer = GlobalBBoxBuffer() # class for recieved bbox maybe we need a double buffer?
        self.rawBuffer = []
        self.parsedBuffer = []
        self.predictions = BBoxPredictor() # class for holding predictions for ID'd bbox
        # Run the fuser at 20Hz
        while not rospy.is_shutdown():
            self.fuserRun50ms()
            self.rate.sleep()

    def fuserRun50ms(self, event=None):
        # process bboxes for this time step
        # read from rawBuffer, convert to BBox class and add to parsedbuffer
        # reset rawBuffer
        self.pub.publish("Hello")

    def nodeCallback(self, data):
        # create new bbox to put in global buffer
        
        # read from subscribers any new data and add to rawBuffer
        pass
        

if __name__ == '__main__':
    try:
        GlobalFuser()
    except rospy.ROSInterruptException:
        pass
