#!/usr/bin/env python
# Class that implemnts a Kalman filter for each bbox
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox

class BBoxPredictor():
	measurement_dim = 3
	state_variables = 6
	delta_t = 0.05 #s
	gate_value = 0.584 #3 DOF 90%
	# gate_value = 2.204 #6 DOF 90%
	new_track_threshold = 500 

	'''
	State Variables: 
		[
			x_pos;
			x_vel;
			y_pos;
			y_vel;
			z_pos;
			z_vel;
		]

	motion model:
		x_pos (k+1) = x_pos(k) + delta_T * x_vel(k)
		x_vel (k+1) = x_vel(k)  
	'''
	
	motion_model = np.array([[1., delta_t, 0., 0., 0., 0.],
							[0., 1., 0., 0., 0., 0.],
							[0., 0., 1., delta_t, 0., 0.],
							[0., 0., 0., 1., 0., 0.],
							[0., 0., 0., 0., 1., delta_t],
							[0., 0., 0., 0., 0., 1.]])

	measuremnt_model = np.array([[1., 0., 0., 0., 0., 0.],
								[0., 0., 1., 0., 0., 0.],
								[0., 0., 0., 0., 1., 0.]])

	default_q = np.array([[0.35, 0., 0., 0., 0., 0.],
						[0., 1., 0., 0., 0., 0.],
						[0., 0., 0.35, 0., 0., 0.],
						[0., 0., 0., 1., 0., 0.],
						[0., 0., 0., 0., 0.35, 0.],
						[0., 0., 0., 0., 0., 1.]])

	default_r = 0.5

	def __init__(self, state_variable_init):
		self.kf = KalmanFilter(dim_x=BBoxPredictor.state_variables, dim_z=BBoxPredictor.measurement_dim)
		self.kf.x = np.array(state_variable_init)
		self.kf.F = BBoxPredictor.motion_model
		self.kf.H = BBoxPredictor.measuremnt_model
		self.kf.P = 1000 #initial covarience matrix
		self.kf.R = BBoxPredictor.default_r
		self.kf.Q = BBoxPredictor.default_q
		self.kf.predict()

	def predict(self):
		self.kf.predict()
		return self.kf.x

	def update(self, sensor_measurement):
		self.kf.update(sensor_measurement)
		self.predict()

	def get_posn(self):
		return (self.kf.x[0], self.kf.x[2], self.kf.x[4])

