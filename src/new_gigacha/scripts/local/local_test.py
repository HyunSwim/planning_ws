#!/usr/bin/env python
#-*- coding:utf-8 -*-
# date : 0703

from __future__ import division, print_function #파이썬3문법을 2에서도 쓸수있게해줌
import serial
import rospy
import math
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
from new_gigacha.msg import Local
import time as t
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
from filterpy.stats import plot_covariance_ellipse
import threading
from std_msgs.msg import Int64,Float32
from geometry_msgs.msg import TwistWithCovarianceStamped

class Localization():
    def __init__(self):
        rospy.init_node('Position_Node', anonymous=False)
        self.pub = rospy.Publisher('/pose', Local, queue_size = 1)

        self.msg = Local()


        self.e_gps = 0
        self.n_gps = 0
        self.u_gps = 0
        self.e_gps_pre = 0
        self.n_gps_pre = 0

        self.e_final = 0
        self.n_final = 0
        self.u_final = 0

        self.e_final_pre = 0
        self.n_final_pre = 0

        self.status = 0

        self.yaw_gps = 0
        self.yaw_imu = 0
        self.yaw_final = 0

        # encoder buffer
        self.c = 0
        self.d = 0
        self.e = 0
        self.f = 0



        self.encoder_flag = 0
        self.dis_gps_flag = 0
        self.dis_DR_flag = 0
        self.dis_DR_enc_left = 0
        self.dis_DR_enc_right = 0 
        self.dis_DR_enc = 0
        self.dis_gps_enc = 0
        
        self.displacement = 0
        self.cur_data_right = 0
        self.displacement_left = 0
        self.displacement_right = 0
        self.diff_left = 0
        self.diff_right = 0
        self.HeadingFrontBackFlg = None # 차량 앞뒤 방향 플래그 , offset업데이트시 사용

        self.e_DR = 0
        self.n_DR = 0
        # self.e_DR = 30.40
        # self.n_DR = 36.72

        self.e_DR = 0
        self.n_DR = 0
        self.position_flag = 0
        self.yaw_flag = 0
        self.offset = 0

        self.t_Time_call = 0 
        self.t_GPS_call = 0.0
        self.t_IMU_call = 0

        self.heading = 0
        self.gps_out = 0
        self.imu_out = 0
        self.headingAcc = 99999999
        self.e_filter = 0
        self.n_filter = 0

        self.cur_point = Point32()
        self.cur_points = PointCloud()
        self.paths = PointCloud()

        self.pub_c = rospy.Publisher('/cur_xy',PointCloud,queue_size = 1)
        self.pub_p = rospy.Publisher('/path',PointCloud,queue_size = 1)

        self.hAcc = 0

        self.vel = TwistWithCovarianceStamped()
        self.vel.header.frame_id = "gps"

        #plot
        self.X1 = []
        self.Y1 = []
        self.X2 = []
        self.Y2 = []
        self.X3 = []
        self.Y3 = []

        self.filter_buffer = []

    def DR(self):
        self.displacement_right = self.cur_data_right
        self.e = self.displacement_right
        self.f = self.displacement_right
        
        # elif self.dis_DR_flag!=0 :
        #     self.c = self.d
        #     self.d = self.displacement_left 

        #     self.e = self.f
        #     self.f = self.displacement_right         

        # if ((self.d - self.c) < -10000000):
        #     self.dis_DR_enc_left = (self.d + (256**4 - self.c))*1.6564/100 # encoder pulse를 m단위로 치환
        # else:
        #     self.dis_DR_enc_left = (self.d - self.c)*1.6564/100
        self.dis_DR_enc_right = (self.cur_data_right - self.e)*1.6564/100

        # if ((self.f - self.e) < -10000000):
        #     self.dis_DR_enc_left = (self.d + (256**4 - self.c))*1.6564/100
        #     self.dis_DR_enc_right = (self.d + (256**4 - self.c))*1.6564/100 
        # else:
        #     self.dis_DR_enc_left = (self.d - self.c)*1.6564/100
        #     self.dis_DR_enc_right = (self.f - self.e)*1.6564/100
        

        # if ((self.f - self.e) > 0):
        #     self.HeadingFrontBackFlg = 1                # 1이면 차량이 직진할 때

        # else:
        #     self.HeadingFrontBackFlg = 0

        self.encoder_velocity = self.dis_DR_enc_right/0.1

        print(f"encoder = {self.encoder_velocity}")

    def Get_Dis_right(self,data):
        res = data.data
        self.cur_data_right = int(res)

        self.DR()
    
    def fix_velocity(self,data):
        # print(1)
        vel_x = data.twist.twist.linear.x
        vel_y = data.twist.twist.linear.y

        gps_velocity = math.sqrt(vel_x**2 + vel_y**2)

        print(f"gps = {gps_velocity}")   

    def GPS_Heading(self, data):
        # self.yaw_gps = (450-(data.heading * 10**(-5)))%360       

        print(data.heading*10**(-5)) 

loc = Localization()
# rospy.Subscriber("/Displacement_right", Int64, loc.Get_Dis_right)
rospy.Subscriber('/ublox_gps/navpvt',NavPVT, loc.GPS_Heading)
# rospy.Subscriber("/ublox_gps/fix_velocity", TwistWithCovarianceStamped, loc.fix_velocity)
rospy.spin()