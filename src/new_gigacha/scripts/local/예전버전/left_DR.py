#!/usr/bin/env python
#-*- coding:utf-8 -*-
# date : 0703

from __future__ import division, print_function #파이썬3문법을 2에서도 쓸수있게해줌
import serial
import rospy
import math
import tf
from std_msgs.msg import Time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavPVT
import matplotlib.pyplot as plt
import numpy as np
from numpy.random import randn
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from scipy.linalg import block_diag
import pymap3d as pm
import time as t
import threading
from std_msgs.msg import Int64

from master_node.msg import Serial_Info
# define
magnet_OFF = [0, 0, 0]
PI = 3.141592

# ENU base coordinates
# # 송도
# base_lat = 37.383784
# base_lon = 126.654310 
# base_alt = 15.4

#kcity
base_lat = 37.239231667
base_lon = 126.773156667
base_alt = 15.400

class Localization():
    def __init__(self):
        rospy.init_node('Position_Node', anonymous=False)
        self.pub = rospy.Publisher('/pose', Odometry, queue_size = 1)

        self.msg = Odometry()


        self.e_gps = None
        self.n_gps = None
        self.u_gps = 0

        self.yaw_imu = 0

        # encoder buffer
        self.c = 0
        self.d = 0
        self.e = 0
        self.f = 0


        self.encoder_flag = 0
        self.dis_DR_flag = 0
        self.dis_DR_enc_left = 0
        self.dis_DR_enc_right = 0 
        self.dis_DR_enc = 0
        
        self.displacement = 0
        self.cur_data_right = 0
        self.displacement_left = 0
        self.displacement_right = 0
        self.diff_left = 0
        self.diff_right = 0

        # self.e_DR = 30.40
        # self.n_DR = 36.72

        self.e_DR = 0
        self.n_DR = 0

        self.heading = 0
        self.gps_out = 0
        self.imu_out = 0
        self.headingAcc = 99999999

        self.cur_point = Point32()
        self.cur_points = PointCloud()
        self.paths = PointCloud()

        self.pub_c = rospy.Publisher('/cur_xy',PointCloud,queue_size = 1)
        self.pub_p = rospy.Publisher('/path',PointCloud,queue_size = 1)


        #plot
        self.X1 = []
        self.Y1 = []
        self.X2 = []
        self.Y2 = []
        self.X3 = []
        self.Y3 = []

    def GPS_call(self,data):        
        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z

        self.e_gps,self.n_gps,self.u_gps = pm.geodetic2enu(lat,lon,alt,base_lat,base_lon,base_alt)

    def msg_write(self,msg):
        self.msg.pose.pose.position.x = float(self.e_DR)
        self.msg.pose.pose.position.y = float(self.n_DR)
        self.msg.pose.pose.position.z = float(self.velocity)
        self.msg.twist.twist.angular.z = self.yaw_imu

    def IMU_call(self,data):
        self.t_IMU_call = t.time()        
        self.yaw_imu = -data.orientation.x
        self.yaw_imu = self.yaw_imu%360
        print(self.yaw_imu )
        


    # #왼쪽 엔코더 콜백함수
    def Get_Dis_left(self, msg):
        self.cur_data_left = msg.encoder   

    def displacement_filter(self):
        if self.encoder_flag == 0:
            self.displacement_left = self.cur_data_left
            self.encoder_flag = 1

        if (abs(self.cur_data_left - self.displacement_left) > 100):
            self.displacement_left = self.displacement_left + self.diff_left
        else:
            self.diff_left = self.cur_data_left - self.displacement_left
            self.displacement_left = self.cur_data_left

    def DR(self):
        if (self.dis_DR_flag == 0) and (self.displacement_left !=0):
            self.c = self.displacement_left
            self.d = self.displacement_left
            self.dis_DR_flag = 1
        
        elif self.dis_DR_flag == 1 :
            self.c = self.d
            self.d = self.displacement_left     

        if ((self.d - self.c) < -10000000):
            self.dis_DR_enc_left = (self.d + (256**4 - self.c))/60.852 #1.6564/100 # encoder pulse를 m단위로 치환
        else:
            self.dis_DR_enc_left = (self.d - self.c)/60.852


        self.dis_DR_enc = self.dis_DR_enc_left # encoder 왼쪽 오른쪽 평균내기

        self.velocity = self.dis_DR_enc/0.1

        self.e_DR += self.dis_DR_enc*math.cos(math.radians(self.yaw_imu))
        self.n_DR += self.dis_DR_enc*math.sin(math.radians(self.yaw_imu))

    def ps(self,X1,Y1,X2,Y2):
        plt.ion()
        # animated_plot = plt.plot(X1, Y1, 'r')[0]
        animated_plot2 = plt.plot(X2, Y2, 'b')[0]
        # animated_plot3 = plt.plot(X3, Y3, 'g')[0]
        ps1=1

        # for i in range(0, len(X1)):
        #     # if i>30:
        #     animated_plot.set_xdata(X1[0:i])
        #     animated_plot.set_ydata(Y1[0:i])
        #     animated_plot2.set_xdata(X2[0:i])
        #     animated_plot2.set_ydata(Y2[0:i])
            # animated_plot3.set_xdata(X3[0:i])
            # animated_plot3.set_xdata(Y3[0:i])

            # else:
                # animated_plot.set_xdata(X1[0:i])
                # animated_plot.set_ydata(Y1[0:i])
                # animated_plot2.set_xdata(X2[0:i])
                # animated_plot2.set_ydata(Y2[0:i])
        plt.draw()
        plt.pause(0.01)

    def Time_call(self,time):
        self.displacement_filter()
        self.DR()
        self.msg_write(self.msg)
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)

        print("왼쪽 엔코더 카운트 : ", self.displacement_left)
        # #Plot
        self.X1.append(self.e_gps)
        self.Y1.append(self.n_gps)

        self.X2.append(self.e_DR)
        self.Y2.append(self.n_DR)

        self.ps(self.X1, self.Y1, self.X2, self.Y2)

loc = Localization()
rospy.Subscriber('/serial', Serial_Info , loc.Get_Dis_left)
rospy.Subscriber('/gps_data/fix',NavSatFix,loc.GPS_call)
rospy.Subscriber('/imu',Imu,loc.IMU_call)
rospy.Subscriber("/timer",Time, loc.Time_call)
rate = rospy.Rate(500)
rospy.spin()

