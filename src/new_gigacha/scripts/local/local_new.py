#!/usr/bin/env python3
from __future__ import division, print_function #파이썬3문법을 2에서도 쓸수있게해줌
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from new_gigacha.msg import Local
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Int64,Float32
from ublox_msgs.msg import NavPVT

import pymap3d

from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise

import numpy as np
import math
import matplotlib.pyplot as plt

PI = math.pi

class Localization():
    def __init__(self):
        rospy.init_node('Localization', anonymous=False)
        self.pub = rospy.Publisher('/pose', Local, queue_size = 1)
        self.msg = Local()

        #Visualization
        self.vis_pub = rospy.Publisher('/vis_pose', PoseStamped, queue_size=1)
        self.vis_msg = PoseStamped()
        self.vis_msg.header.frame_id = "map"
        
        #KCity
        # self.lat_origin = 37.239231667
        # self.lon_origin = 126.773156667
        # self.alt_origin = 15.400
        
        # #Songdo
        # self.lat_origin = 37.3851693 
        # self.lon_origin = 126.6562271
        # self.alt_origin = 15.4

        self.yaw_gps = 0
        self.yaw_imu = 0
        self.yaw_rate = 0
        self.yaw_filter = 0
        
        self.X1 = []
        self.X2 = []

        rospy.Subscriber("/simul_gps", Pose, self.gps_call_back)
        rospy.Subscriber("/simul_imu", Pose, self.imu_call_back)
        
        rospy.Subscriber('/ublox_gps/navpvt',NavPVT, self.gps_Heading)
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.gps_call_back)
        rospy.Subscriber("/imu", Imu, self.imu_call_back)

##########KALMAN FILTER##############
    def all_filter(self):
        f = KalmanFilter(dim_x=2,dim_z=2)
        dt = 0.1
        
        f.F = np.array([[1,dt],
                        [0,1]])
        
        # Noise (l)
        q = Q_discrete_white_noise(dim=1, dt=0.1, var=0.001)
        f.Q = block_diag(q, q)

        # Observe Matrix H
        f.H = np.array([[1,dt],
                        [0,1]])

        f.R = np.array([[self.cov_x_gps,0],
                        [0,0]])

        #######INITIAL CONDITIONS########
        f.x = np.array([[0, 0]]).T
        f.P = np.eye(2)
        
        return f

###############Get ENU Coordinate and implement KALMAN FILTER###########
    def gps_call_back(self, data):
        self.msg.x, self.msg.y, _ = pymap3d.geodetic2enu(data.latitude, data.longitude, self.alt_origin, \
                                            self.lat_origin , self.lon_origin, self.alt_origin)

        self.cov_x_gps = data.position_covariance[4]
        self.cov_y_gps = data.position_covariance[0]
        self.cov_xy_gps = data.position_covariance[1]

        #kalman filter 
        zs = np.array([self.yaw_gps, self.yaw_rate]).T
        
        filter = self.all_filter()
        filter.predict()
        filter.update(zs)
        
        self.yaw_filter = filter.x[0]

        self.X1.append(self.yaw_imu)

        self.X2.append(self.yaw_filter)

        self.ps(self.X1,self.X2)

#################################################
    def ps(self,X1,X2):
        plt.ion()
        animated_plot = plt.plot(X1, 'r')[0]
        animated_plot2 = plt.plot(X2, 'b')[0]
        ps1=1

        for i in range(0, len(X1)):
            if i>30:
                animated_plot.set_xdata(X1[0:i])
                animated_plot2.set_xdata(X2[0:i])

            else:
                animated_plot.set_xdata(X1[0:i])
        plt.draw()
        plt.pause(0.01)
##################################################

################Calculate heading###################
    def imu_call_back(self, data):
        self.vis_msg.pose.orientation = data.orientation
        self.yaw_rate = -data.angular_velocity.z

        orientation_q = data.orientation
        roll, pitch, yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        self.yaw_imu = np.rad2deg(yaw)
        
    def gps_Heading(self, data):
        self.yaw_gps = data.heading

###############Quaternion to Euler####################
    def euler_from_quaternion(self,x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


    def main(self):
        self.msg.heading = self.yaw_filter
        self.pub.publish(self.msg)

        self.vis_msg.pose.position.x = self.msg.x
        self.vis_msg.pose.position.y = self.msg.y
        self.vis_msg.header.stamp = rospy.Time.now()
        self.vis_pub.publish(self.vis_msg)

        print("self.yaw_imu:{}".format(self.yaw_imu))

if __name__ == '__main__':
    loc = Localization()
    rate = rospy.Rate(100)
 
    while not rospy.is_shutdown():
        loc.main()
        rate.sleep()