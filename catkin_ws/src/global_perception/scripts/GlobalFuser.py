#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
import message_filters as mf
import numpy as np
from jsk_recognition_msgs.msg import BoundingBoxArray
from BBoxPredictor import BBoxPredictor

NODE_RATE = 20 # Hz
POSITION_TOLERANCE = 2. #m
TIME_THRESHOLD = 150
TOL = 0.5

class GlobalFuser():
    def __init__(self):
        rospy.init_node('global_perception', anonymous=False)
        self.rate = rospy.Rate(NODE_RATE)

        sub_n1 = mf.Subscriber('node1/bounding_boxes', BoundingBoxArray)
        sub_n2 = mf.Subscriber('node2/bounding_boxes', BoundingBoxArray)

        sub_n12 = mf.Subscriber('node1/bounding_boxes', BoundingBoxArray)
        # sub_n12.registerCallback(self.testCallback)
        # time_synch = mf.TimeSynchronizer([sub_n1, sub_n2], 10)
        sub_n1.registerCallback(self.nodeCallback)
        sub_n2.registerCallback(self.nodeCallback)
        
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

        ids_updated = []
        print(self.tracked_bboxes.keys())
        # update tracker with ids recieved
        for node in self.master_buffer.keys():
            for bbox in self.master_buffer[node].boxes:
                if str(bbox.value) in self.tracked_bboxes.keys():
                    id_key = str(bbox.value)
                    if abs(self.tracked_bboxes[id_key]['latest_bbox'].dimensions.x - bbox.dimensions.x)>TOL:
                        self.tracked_bboxes[id_key]['timesteps_missed'] = 0
                        self.tracked_bboxes[id_key]['latest_bbox'].pose = bbox.pose
                        self.tracked_bboxes[id_key]['latest_bbox'].header = bbox.header
                        self.tracked_bboxes[id_key]['latest_bbox'].value = float(id_key)
                    else:
                        self.tracked_bboxes[id_key]['latest_bbox'] = bbox
                    bbox_pos = [bbox.pose.position.x,
                                bbox.pose.position.y,
                                bbox.pose.position.z]
                    self.tracked_bboxes[id_key]['kf'].update(bbox_pos)
                    ids_updated.append(id_key)
                    continue

                # we dont recognize this bbox check all predictions first
                found_match = False
                min_gate_value = 10000000000000000
                for id in self.tracked_bboxes.keys():
                    # gating try
                    self.tracked_bboxes[id]['kf'].predict()
                    predicted_pos = self.tracked_bboxes[id]['kf'].get_posn()
                    id_residual = [0, 0, 0]
                    id_residual[0] = bbox.pose.position.x - predicted_pos[0]
                    id_residual[1] = bbox.pose.position.y - predicted_pos[1]
                    id_residual[2] = bbox.pose.position.z - predicted_pos[2]
                    id_residual_np = np.array(id_residual)

                    s_temp = np.dot(BBoxPredictor.measuremnt_model, self.tracked_bboxes[id]['kf'].kf.P)
                    S = np.dot(s_temp, np.transpose(BBoxPredictor.measuremnt_model))+self.tracked_bboxes[id]['kf'].kf.R
                    S_inv = np.linalg.inv(S)
                    gate_value = np.dot(np.dot(np.transpose(id_residual_np), S_inv), id_residual_np)
                    
                    if gate_value < min_gate_value:
                        min_gate_value = gate_value
                    
                    print(gate_value, id, bbox.value)
                    if gate_value < BBoxPredictor.gate_value:
                        print("Found Match")
                        print(gate_value, id, bbox.value)
                        # we have seen this ID, update it with latest bbox and kf
                        self.tracked_bboxes[id]['timesteps_missed'] = 0
                        if abs(self.tracked_bboxes[id]['latest_bbox'].dimensions.x - bbox.dimensions.x)>TOL:
                            self.tracked_bboxes[id]['timesteps_missed'] = 0
                            self.tracked_bboxes[id]['latest_bbox'].pose = bbox.pose
                            self.tracked_bboxes[id]['latest_bbox'].header = bbox.header
                        else:
                            self.tracked_bboxes[id]['latest_bbox'] = bbox
                        self.tracked_bboxes[id]['latest_bbox'].value = float(id)
                        bbox_pos = [bbox.pose.position.x,
                                    bbox.pose.position.y,
                                    bbox.pose.position.z]
                        self.tracked_bboxes[id]['kf'].update(bbox_pos)
                        ids_updated.append(id)
                        found_match = True
                    
                
                if not found_match and min_gate_value > BBoxPredictor.new_track_threshold:
                    # its a new id to add to the tracker
                    id_key = str(bbox.value)
                    print("new id to tracker id: " + id_key)
                    self.tracked_bboxes[id_key] = {}
                    self.tracked_bboxes[id_key]['timesteps_missed'] = 0
                    self.tracked_bboxes[id_key]['latest_bbox'] = bbox
                    bbox_pos = [bbox.pose.position.x,
                                0,
                                bbox.pose.position.y,
                                0,
                                bbox.pose.position.z,
                                0]
                    self.tracked_bboxes[id_key]['kf'] = BBoxPredictor(bbox_pos)
                    ids_updated.append(id_key)

        # check all the ids not updated, update with prediction instead
        ids_not_updated = list(set(self.tracked_bboxes.keys()) - set(ids_updated))
        for id in ids_not_updated:
            self.tracked_bboxes[id]['timesteps_missed'] += 1
            if self.tracked_bboxes[id]['timesteps_missed'] > TIME_THRESHOLD:
                pass
                # bbox has been gone too long delete it from tracker
                print("deleting id " + str(id))
                del self.tracked_bboxes[id]
            else:
                # else use its prediction for now
                self.tracked_bboxes[id]['kf'].predict()
                predicted_pos = self.tracked_bboxes[id]['kf'].get_posn() 
                self.tracked_bboxes[id]['latest_bbox'].pose.position.x = predicted_pos[0]
                self.tracked_bboxes[id]['latest_bbox'].pose.position.y = predicted_pos[1]
                self.tracked_bboxes[id]['latest_bbox'].pose.position.z = predicted_pos[2]
            
        # at this point everything left in tracker should have its latest bbox published
        fused_buffer = BoundingBoxArray()
        fused_buffer.header.frame_id = 'map'
        for id in self.tracked_bboxes.keys():
            fused_buffer.boxes.append(self.tracked_bboxes[id]['latest_bbox'])
        
        self.pub.publish(fused_buffer)


    def nodeCallback(self, data_n1):
        # Add all bboxes to raw buffer
        self.master_buffer['node1'].header = data_n1.header
        self.master_buffer['node1'].boxes = data_n1.boxes
        # print(len(data_n1.boxes))
        # if len(data_n1.boxes) == 3:
        #     print("found it")
        # self.master_buffer['node2'].header = data_n2.header
        # self.master_buffer['node2'].boxes = data_n2.boxes
        # print(len(data_n2.boxes))
        

if __name__ == '__main__':
    try:
        GlobalFuser()
    except rospy.ROSInterruptException:
        pass
