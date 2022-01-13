#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox

# class GlobalBBox():
# 	def __init__(self):
# 		# self.nodeNum
#         # self.id
#         # self.data - store bbox msg here
# 		pass

# 	def updateID(self):
# 		pass

class GlobalBBoxBufferClass():
    def __init__(self):
        self.unfilteredBuffer = list()

    def addNewBBox(self, node_data):
    	self.unfilteredBuffer.append(node_data)

    def resetBuffer(self):
    	self.unfilteredBuffer = []