#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox


class GlobalBBox():
	def __init__(self):
		# self.nodeNum
        # self.id
        # self.data - store bbox msg here
		pass

	def updateID(self):
		pass


# class GlobalBBoxBuffer():
#     def __init__(self):
#         self.unfilteredBuffer = list()

#     def addNewBBox(self):
#     	pass