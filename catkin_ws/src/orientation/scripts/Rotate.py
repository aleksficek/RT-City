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
        np_pc2 = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_data)
        rotated_np_pc2 = np.matmul(np_pc2, self.rotation_matrx)
        final_output = self.xyz_array_to_pointcloud2(rotated_np_pc2)
        self.pub.publish(final_output)

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
        return A_R_B

    def xyz_array_to_pointcloud2(self, points, stamp=None, frame_id=None):
        '''
        Create a sensor_msgs.PointCloud2 from an array
        of points.
        '''
        msg = PointCloud2()
        if stamp:
            msg.header.stamp = stamp
        if frame_id:
            msg.header.frame_id = frame_id
        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12*points.shape[0]
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = np.asarray(points, np.float32).tostring()
        return msg


if __name__ == '__main__':
    try:
        Orient()
    except rospy.ROSInterruptException:
        pass
