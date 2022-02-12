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
        ids_updated = []
        # update tracker with ids recieved
        for node in self.master_buffer:
            for bbox in self.master_buffer[node].boxes:
                if str(bbox.value) in self.tracked_bboxes:
                    # we have seen this ID, update it with latest bbox and kf
                    id_key = str(bbox.value)
                    self.tracked_bboxes[id_key]['latest_bbox'] = bbox
                    bbox_pos = [bbox.pose.position.x,
                                bbox.pose.position.y,
                                bbox.pose.position.z]
                    self.tracked_bboxes[id_key]['kf'].update(bbox_pos)
                    ids_updated.append(id_key)
                else:
                    # we dont recognize this bbox check all predictions first 
                    ids_to_check = list(set(self.tracked_bboxes.keys())- set(ids_updated))
                    if len(ids_to_check) == 0:
                        # its a new id to add to the tracker
                        id_key = str(bbox.value)
                        self.tracked_bboxes[id_key] = {}
                        self.tracked_bboxes[id_key]['latest_bbox'] = bbox
                        bbox_pos = [bbox.pose.position.x,
                                    1,
                                    bbox.pose.position.y,
                                    1,
                                    bbox.pose.position.z,
                                    0]
                        self.tracked_bboxes[id_key]['kf'] = BBoxPredictor(bbox_pos)
                        ids_updated.append(id_key)
                        break #break ids to check
                    else:
                        for id in ids_to_check:
                            predicted_pos = self.tracked_bboxes[id]['kf'].get_posn()
                            diff_x = abs(bbox.pose.position.x - predicted_pos[0])
                            diff_y = abs(bbox.pose.position.y - predicted_pos[1])
                            diff_z = abs(bbox.pose.position.z - predicted_pos[2])
                            if diff_x < POSITION_TOLERANCE and diff_y < POSITION_TOLERANCE and diff_z < POSITION_TOLERANCE:
                                # we recognize this bbox lets update its pos in tracker
                                self.tracked_bboxes[id]['latest_bbox'] = bbox
                                bbox_pos = [bbox.pose.position.x,
                                            bbox.pose.position.y,
                                            bbox.pose.position.z]
                                self.tracked_bboxes[id]['kf'].update(bbox_pos)
                                ids_updated.append(id)
                                break #break ids to check
                            else:
                                # its a new id to add to the tracker
                                id_key = str(bbox.value)
                                self.tracked_bboxes[id_key]['latest_bbox'] = bbox
                                bbox_pos = [bbox.pose.position.x,
                                            bbox.pose.position.y,
                                            bbox.pose.position.z]
                                self.tracked_bboxes[id_key]['kf'].update(bbox_pos)
                                ids_updated.append(id_key)
                                break #break ids to check

        # check all the ids not updated, update with prediction instead
        ids_not_updated = list(set(self.tracked_bboxes.keys()) - set(ids_updated))
        for id in ids_not_updated:
            self.tracked_bboxes[id][timesteps_missed] +=1
            if self.tracked_bboxes[id][timesteps_missed] > TIME_THRESHOLD:
                # bbox has been gone to long delete it from tracker
                del self.tracked_bboxes[id]
            else:
                # else use its prediction for now
                predicted_pos = self.tracked_bboxes[id]['kf'].get_posn() 
                self.latest_bbox.pose.position.x = predicted_pos[0]
                self.latest_bbox.pose.position.y = predicted_pos[1]
                self.latest_bbox.pose.position.z = predicted_pos[2]
            
        # at this point everything left in tracker should have its latest bbox published
        fused_buffer = BoundingBoxArray()
        fused_buffer.header.frame_id = 'map'
        # print(self.tracked_bboxes)
        for id in self.tracked_bboxes:
            fused_buffer.boxes.append(self.tracked_bboxes[id]['latest_bbox'])
        
        self.pub.publish(fused_buffer)


    def nodeCallback(self, data_n1, data_n2):
        # Add all bboxes to raw buffer
        self.master_buffer['node1'].header = data_n1.header
        self.master_buffer['node1'].boxes = data_n1.boxes
        
        self.master_buffer['node2'].header = data_n1.header
        self.master_buffer['node2'].boxes = data_n1.boxes
        print("here")
        # run predictions on all bboxes from last time step
        self.runAllPredictions()

    def runAllPredictions(self):
        for id in self.tracked_bboxes:
            self.tracked_bboxes[id]['kf'].predict()
        

if __name__ == '__main__':
    try:
        GlobalFuser()
    except rospy.ROSInterruptException:
        pass
