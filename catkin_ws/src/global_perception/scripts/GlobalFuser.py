#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
import message_filters as mf
from jsk_recognition_msgs.msg import BoundingBoxArray
from BBoxPredictor import BBoxPredictor

NODE_RATE = 20 # Hz
POSITION_TOLERANCE = 0.15 #m

class GlobalFuser():
    def __init__(self):
        rospy.init_node('global_perception', anonymous=False)
        self.rate = rospy.Rate(NODE_RATE)

        self.sub_n1 = mf.Subscriber('node1/bounding_boxes', BoundingBoxArray)
        self.sub_n2 = mf.Subscriber('node2/bounding_boxes', BoundingBoxArray)
        self.time_synch = mf.TimeSynchronizer([self.sub_n1, self.sub_n2], 10)
        self.time_synch.registerCallback(self.nodeCallback)
        
        self.master_buffer = {}
        self.master_buffer['node1'] = BoundingBoxArray()
        self.master_buffer['node2'] = BoundingBoxArray()

        ''' 
        master list of tracked bboxes ('id': (latest_BBox_msg, kalman_filter, timesteps_missed))

        New Idea:
            'id':
                'latest_bbox': BoundingBox()
                'kf': BBoxPredictor()
                'timesteps_missed': count (just an int value)
        '''
        self.tracked_bboxes = {}

        self.pub = rospy.Publisher('global_fused_bboxes', BoundingBoxArray, queue_size=10)
        
        # Run the fuser at 20Hz
        while not rospy.is_shutdown():
            self.fuserRun50ms()
            self.rate.sleep()

    def fuserRun50ms(self, event=None):

        '''
        Basic Idea:
            1. first check if we have a new ID
            2. if new ID check all predictions to see if any match within a tolerance
                - reassign ID
                - update bbox KF with latest measurements
                - keep track of the IDs we have updated 
                else add a new track and ID to global tracker
                - update bbox KF with latest measurements
                - keep track of the IDs we have updated 

            else if we dont have a new ID
                - update bbox KF with latest measurements
                - keep track of the IDs we have updated 
            
            3. check what IDs werent updated, if ID hasnt been updated add to 1 to count
                - if reached time threshold we drop bbox ID from dictionary

            4. update latest bbox with KF position(maybe)
            5. publish all latest bboxes from dictionary
        '''

        # old code
        fused_buffer = BoundingBoxArray()
        fused_buffer.header.stamp = rospy.Time.now()
        fused_buffer.header.frame_id = 'lidar/node1_lidar'

        if (len(self.master_buffer['node1'].boxes)>0):

            # check for new ID first and simple overlap
            for node in self.master_buffer:
                for bbox in self.master_buffer[node].boxes:
                    box_already_there = False   
                    if len(fused_buffer.boxes) == 0:
                        fused_buffer.boxes.append(bbox)
                    for compare_box in fused_buffer.boxes:
                        diff_x = abs(bbox.pose.position.x - compare_box.pose.position.x)
                        diff_y = abs(bbox.pose.position.y - compare_box.pose.position.y)
                        diff_z = abs(bbox.pose.position.z - compare_box.pose.position.z)
                        if diff_x < POSITION_TOLERANCE and diff_y < POSITION_TOLERANCE and diff_z < POSITION_TOLERANCE:
                            box_already_there = True
                            break
                    if not box_already_there:
                        fused_buffer.boxes.append(bbox)
        # end old code

            # look at bbox kfs predicted to this time step
            ids_to_publish = []
            for id in self.tracked_bboxes:
                # get pos and compare to new bboxes to add
                prediction_pos = self.tracked_bboxes[id][1].get_posn()
                # if within range, assign it to bbox placeholder in dictionary, update kf

        for id in ids_to_publish:
            fused_buffer.boxes.append(self.tracked_bboxes[id][1])
        
        self.pub.publish(fused_buffer)


    def nodeCallback(self, data_n1, data_n2):
        # Add all bboxes to raw buffer
        self.master_buffer['node1'].header = data_n1.header
        self.master_buffer['node1'].boxes = data_n1.boxes
        self.master_buffer['node2'].header = data_n1.header
        self.master_buffer['node2'].boxes = data_n1.boxes

        # run predictions on all bboxes from last time step
        self.runAllPredictions()

    def runAllPredictions(self):
        for kf in self.tracked_bboxes():
            kf.predict()
        

if __name__ == '__main__':
    try:
        GlobalFuser()
    except rospy.ROSInterruptException:
        pass
