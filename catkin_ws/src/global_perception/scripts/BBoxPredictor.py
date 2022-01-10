#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox

'''
Notes:
- needs dictonary data structure to hold predictions per ID
- probably needs some sort of struct to hold prediction info? unless all we care about is a radius
- subscribe to some sort of speed channel for bboxes
- paramaters to associate bbox in checkPredictions

'''

class BBoxPredictor():
	def __init__(self):
		pass

	def predict(self):
		pass

	def checkPredictions(self):
		# given buffer of bboxes and predictions check if any need to be ReID'd
		pass
