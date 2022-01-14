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

# define
magnet_OFF = [0, 0, 0]
PI = math.pi

# ENU base coordinates
# 송도
base_lat = 37.3851693
base_lon = 126.6562271
base_alt = 15.4
    

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

        #plot
        self.X1 = []
        self.Y1 = []
        self.X2 = []
        self.Y2 = []
        self.X3 = []
        self.Y3 = []

        self.filter_buffer = []

    def all_filter(self):
        f = KalmanFilter(dim_x=4,dim_z=4)
        dt = 0.1

        # State Transition Matrix F
        f.F = np.array([[1,dt,0,0],
                        [0,1,0,0],
                        [0,0,1,dt],
                        [0,0,0,1]])

        # Noise (l)
        q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.001)                # moise Q 상태변수에 영향을 주는 잡음
        f.Q = block_diag(q, q)


        # Observe Matrix H
        f.H = np.array([[1,0,0,0],
                        [1,0,0,0],
                        [0,0,1,0],
                        [0,0,1,0]])


        # Obeservation Noise R (v)
        cov_x_enc = 0.01#0.0031360000000000003
        cov_y_enc = 0.01#0.0031360000000000003

        cov_x_gps = 0.01 #0.0007199025610000001
        cov_y_gps = 0.01 #0.0007199025610000001


        f.R = np.array([[cov_x_gps, 0, 0, 0],                                 # 측정값의 잡음 vk의 행렬
                    [0, cov_x_enc, 0, 0],
                    [0, 0, cov_y_gps, 0],
                    [0, 0, 0, cov_y_enc]])  



        #######INITIAL CONDITIONS########
        f.x = np.array([[0, 0, 0, 0]]).T
        f.P = np.eye(4) * 500.
        
        return f


    def gps_check(self): #gps신뢰도 판단하는 부분 
        if self.hAcc > 210:
            self.position_flag = 1
        else:
            self.position_flag = 0

    def msg_write(self,msg):
        self.msg.x = float(self.e_final)
        self.msg.y = float(self.n_final)
        self.msg.velocity = float(self.velocity)
        self.msg.heading = self.yaw_final
        self.e_final_pre = self.e_final
        self.n_final_pre = self.n_final

    def GPS_call(self,data):
        self.t_GPS_call = t.time()

        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z

        self.status = data.status.status #RTK status

        self.e_gps,self.n_gps,self.u_gps = pm.geodetic2enu(lat,lon,alt,base_lat,base_lon,base_alt)
        
        #kalman filter 
        zs = (self.e_gps,self.e_DR,self.n_gps,self.n_DR)
        
        filter = self.all_filter()
        filter.predict()
        filter.update(zs)

        self.e_filter = filter.x[0]
        self.n_filter = filter.x[2]

        #gps 신뢰도 판단
        self.gps_check()
        self.e_gps_pre = self.e_gps
        self.n_gps_pre = self.n_gps

        #Plot
        self.X1.append(self.e_gps)
        self.Y1.append(self.n_gps)

        self.X2.append(self.e_DR)
        self.Y2.append(self.n_DR)

        self.X3.append(self.e_filter)
        self.Y3.append(self.n_filter)


        self.ps(self.X1, self.Y1, self.X2, self.Y2, self.X3, self.Y3)

    def GPS_Heading(self, data):
        self.yaw_gps = (450-(data.heading * 10**(-5)))%360
        self.hAcc = data.hAcc
        self.headingAcc =data.headAcc
        
    def IMU_call(self,data):
        self.t_IMU_call = t.time()        
        self.yaw_imu = -data.orientation.x
        self.yaw_imu = self.yaw_imu%360
        


    def Get_Dis_left(self, data):
###################################################### 이동평균필터 수정하기 ##########################
        # self.filter_buffer.append(self.dis_diff)

        # if abs(self.data_pre - data.data)>100:
        #     pass
        # else:
        #     if len(self.filter_buffer)>2:
        #         del self.filter_buffer[0]
        #         self.displacement = self.displacement_ppc - self.dis_diff + (self.filter_buffer[0] + self.filter_buffer[1])/2
        #     else:
        #         self.displacement = self.displacement_ppc
####################################################################################################

        cur_data_left = data.data
        
        if self.encoder_flag == 0:
            self.displacement_left = cur_data_left
            self.displacement_right = self.cur_data_right
            self.encoder_flag = 1

        if (abs(cur_data_left - self.displacement_left) > 100):
            self.displacement_left = self.displacement_left + self.diff_left
        else:
            self.diff_left = cur_data_left - self.displacement_left
            self.displacement_left = cur_data_left

        
        if (abs(self.cur_data_right - self.displacement_right) > 100):
            self.displacement_right = self.displacement_right + self.diff_right
        else:
            self.diff_right = self.cur_data_right - self.displacement_right            # 여기서 displacement_right는 이전값을 의미한다
            self.displacement_right = self.cur_data_right                              # 여기서 displacement_right는 현재값을 의미한다



    #오른쪽 엔코더 콜백함수
    def Get_Dis_right(self, data):
        res = data.data
        self.cur_data_right = int(res)



    #Dead Reckoning 
    def DR(self):
        if (self.dis_DR_flag == 0) and (self.displacement_left !=0):
            self.c = self.displacement_left
            self.d = self.displacement_left
            self.e = self.displacement_right
            self.f = self.displacement_right
            self.dis_DR_flag = 1
        
        elif self.dis_DR_flag!=0 :
            self.c = self.d
            self.d = self.displacement_left 

            self.e = self.f
            self.f = self.displacement_right         

        if ((self.d - self.c) < -10000000):
            self.dis_DR_enc_left = (self.d + (256**4 - self.c))*1.6564/100 # encoder pulse를 m단위로 치환
        else:
            self.dis_DR_enc_left = (self.d - self.c)*1.6564/100
            self.dis_DR_enc_right = (self.f - self.e)*1.6564/100

        if ((self.f - self.e) < -10000000):
            self.dis_DR_enc_left = (self.d + (256**4 - self.c))*1.6564/100
            self.dis_DR_enc_right = (self.d + (256**4 - self.c))*1.6564/100 
        else:
            self.dis_DR_enc_left = (self.d - self.c)*1.6564/100
            self.dis_DR_enc_right = (self.f - self.e)*1.6564/100
        

        if ((self.f - self.e) > 0):
            self.HeadingFrontBackFlg = 1                # 1이면 차량이 직진할 때

        else:
            self.HeadingFrontBackFlg = 0

        self.velocity = self.dis_DR_enc_right/0.1

        self.dis_DR_enc = (self.dis_DR_enc_right + self.dis_DR_enc_left)/2 # encoder 왼쪽 오른쪽 평균내기

        self.e_DR += self.dis_DR_enc*math.cos(self.yaw_final*PI/180)
        self.n_DR += self.dis_DR_enc*math.sin(self.yaw_final*PI/180)

        self.yaw_check()


    def yaw_check(self): # 차량이 똑바로 직진시, heading 정확도가 200000보다 낮을 
        if ((self.dis_DR_enc_right - self.dis_DR_enc_left) < 0.03) and self.HeadingFrontBackFlg == 1 and self.headingAcc < 200000:
            self.yaw_flag = 1
        else:
            self.yaw_flag = 0

    def decide_heading(self):
        if self.imu_out == 0:				# imu 정상
            if self.gps_out == 0:			# gps 정상
                if self.yaw_flag == 1: # gps o,imu o & 신뢰도 o & 앞으로 갈 때
                    self.offset = self.yaw_gps - self.yaw_imu 
                    self.yaw_final = self.yaw_imu + self.offset

                else:                                                        # gps o,imu o & 신뢰도 x or #gps o,imu o & 뒤로갈 때
                    self.yaw_final = self.yaw_imu + self.offset

            else:
                self.yaw_final = self.yaw_imu + self.offset                  # gps가 꺼졌을 때
        
        else:
            if self.gps_out == 0:                                            # imu가 꺼졌을 떄   
                self.yaw_final = self.yaw_gps
            else:                                                            # imu / gps가 꺼졌을 때
                pass

    def decide_position(self):
        self.DR()
        if self.gps_out == 0:                                               # gps o
            if self.position_flag == 0:                                     # 신뢰도 o
                self.e_final = self.e_gps
                self.n_final = self.n_gps
                
                self.e_DR = self.e_final
                self.n_DR = self.n_final
                print("gps 주행 중")      

            else:                                                           # 신뢰도 o
                self.e_final = self.e_filter
                self.n_final = self.n_filter
                print("sensor fusion 주행 중")
                
        else:                                                               # gps x
            self.e_final = self.e_DR
            self.n_final = self.n_DR
            print("dead Reckoning 주행 중")


    def ps(self,X1,Y1,X2,Y2,X3,Y3):
        plt.ion()
        animated_plot = plt.plot(X1, Y1, 'r')[0]
        animated_plot2 = plt.plot(X2, Y2, 'b')[0]
        animated_plot3 = plt.plot(X3, Y3, 'g')[0]
        ps1=1

        for i in range(0, len(X1)):
            if i>30:
                animated_plot.set_xdata(X1[0:i])
                animated_plot.set_ydata(Y1[0:i])
                animated_plot2.set_xdata(X2[0:i])
                animated_plot2.set_ydata(Y2[0:i])
                animated_plot3.set_xdata(X3[0:i])
                animated_plot3.set_xdata(Y3[0:i])

            else:
                animated_plot.set_xdata(X1[0:i])
                animated_plot.set_ydata(Y1[0:i])
                animated_plot2.set_xdata(X2[0:i])
                animated_plot2.set_ydata(Y2[0:i])
        plt.draw()
        plt.pause(0.01)

    def Time_call(self,time):
        self.t_Time_call = t.time()

        # if self.t_Time_call - self.t_GPS_call > 0.2:
        #     self.gps_out = 1
        # else:
        #     self.gps_out = 0

        if self.t_Time_call - self.t_IMU_call > 0.5:
            self.imu_out = 1
        else:
            self.imu_out = 0  
        
        self.decide_heading()
        self.decide_position()
        
        self.msg_write(self.msg)
        print("I:",round(self.yaw_imu),"  g:",round(self.yaw_gps),"   O: ",round(self.offset),"  f:",round(self.yaw_final),"   FBflg",self.HeadingFrontBackFlg, "    y_check", self.yaw_flag)
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)   



loc = Localization()
rospy.Subscriber('/Displacement', Int64 , loc.Get_Dis_left)
rospy.Subscriber("/Displacement_right", Int64, loc.Get_Dis_right)
rospy.Subscriber('/gps_data/fix',NavSatFix,loc.GPS_call)
rospy.Subscriber('/gps_data/navpvt',NavPVT, loc.GPS_Heading)
rospy.Subscriber('/imu',Imu,loc.IMU_call)
rospy.Subscriber("/timer",Time, loc.Time_call)
rate = rospy.Rate(500)
rospy.spin()

