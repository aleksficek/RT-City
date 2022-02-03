#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
from math import pi, cos, sin
import numpy as np

NODE_RATE = 20 # Hz
ROLL, PITCH, YAW = 20, 20, 20


class Orient():
    def __init__(self):
        rospy.init_node('orientation', anonymous=False)
        self.rate = rospy.Rate(NODE_RATE)
        self.sub = rospy.Subscriber('rslidar_points_front', PointCloud2, self.callback)
        self.pub = rospy.Publisher('rotated_point_cloud', PointCloud2, queue_size=10)
        self.rotation_matrx = self.rot_matrix(ROLL, PITCH, YAW)
        rospy.spin()

    def callback(self, pc2_data):
        header = Header()
        header.frame_id = "map"
        header.stamp = rospy.Time.now()

        numpy_pc2 = ros_numpy.point_cloud2.get_xyz_points(pc2_data)

        # rotation_matrix = self.rot_matrix


        # rospy.loginfo(numpy_pc2)

        # rotated_np_pc2 = numpy_pc2

        # output = pc2.create_cloud(header, fields, rotated)
        # self.pub.publish(output)

    def rot_matrix(self, roll, pitch, yaw): 
        ''' Compute Rotation Matrix from {B} to {A} = A_R_B given RPY angles using 
            {A} as fixed axis about which RPY of {B} is given: 
            Roll is about x axis, Pitch about y axis, and Yaw about z axis. 

            Inputs: Roll, pitch, and yaw angles in degrees 
            Outputs: A_R_B (3x3) 
        ''' 
        alpha = yaw*pi/180; beta = pitch*pi/180; gamma = roll*pi/180 
        Rz = np.array([[cos(alpha), -sin(alpha), 0],[sin(alpha),cos(alpha),0],[0,0,1]])
        Ry = np.array([[cos(beta), 0, sin(beta)],[0,1,0],[-sin(beta),0,cos(beta)]])
        Rx = np.array([[1,0,0],[0,cos(gamma),-sin(gamma)],[0,sin(gamma),cos(gamma)]])
         
        A_R_B = np.matmul(np.matmul(Rz, Ry), Rx)
        # A_R_B = Rz@Ry@Rx
        # To check with final form of principal rotation matrix multiplication 
        # R_XYZ = np.array([[cos(alpha)*cos(beta),cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma),cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma)], 
        # [sin(alpha)*cos(beta),sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma),sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma)], 
        # [-sin(beta),cos(beta)*sin(gamma),cos(beta)*cos(gamma)]]) 
        # print("\nR_XYZ:\n",R_XYZ) 
        return A_R_B


if __name__ == '__main__':
    try:
        Orient()
    except rospy.ROSInterruptException:
        pass
