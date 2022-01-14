#!/usr/bin/env python
#-*- coding:utf-8 -*-
# date : 0802
# 시리얼 노드가 바뀌면서 데드레코닝 sync가 뒤틀려버림 이걸 수정해야함.
# 어떻게 할거냐. GPS rate도 중가가 필요하다고 판단. (기존 GPS Hz 5) > DR을 통한 20Hz로 헤르쯔를 올릴거임
# time.py의 속도를 20hz로 증가시킴. 거기에서 데드레코닝을 꾸준히 돌림

# mode1 (GPS ON) = GPS의 sync를 살려서 중간 중간 DR로 보간하여 위치를 추적함.
# mode2 (GPS OFF) = time.py의 속도에 맞추어서 DR실행

# GPS의 신호 사이에 3개의 DR이 들어감.
# time, GPS, sync를 맞추기는 어려움, 주기적인 GPS를 가지고 SYnc를 맞추자
# 수정해야 할 것 : count확인하기, encoder오류값 찾기 = 최대 속도 주행시 enc_counter의 범위를 알아봐야함.

from __future__ import division, print_function #파이썬3문법을 2에서도 쓸수있게해줌
import serial
import rospy
import math
import tf
import time
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
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
from filterpy.stats import plot_covariance_ellipse
import threading
from std_msgs.msg import Int64

from master_node.msg import Serial_Info
# define
magnet_OFF = [0, 0, 0]
PI = 3.141592

# ENU base coordinates
# 송도
base_lat = 37.383784
base_lon = 126.654310 
base_alt = 15.4
    
# kcity
# base_lat = 37.239231667
# base_lon = 126.773156667 
# base_alt = 15.4



class Localization():
    def __init__(self):
        rospy.init_node('Position_Node2', anonymous=False)
        self.pub = rospy.Publisher('/pose', Odometry, queue_size = 1)

        self.msg = Odometry()


        self.e_gps = 0
        self.n_gps = 0
        self.u_gps = 0
        self.e_gps_pre = 0
        self.n_gps_pre = 0

        self.e_final = None
        self.n_final = None
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

        self.velocity = 0
        self.encoder_flag = 0
        self.dis_gps_flag = 0
        self.dis_DR_flag = 0
        self.dis_DR_enc_left = 0
        self.dis_DR_enc_right = 0 
        self.dis_DR_enc = 0
        self.dis_gps_enc = 0
        
        self.displacement = 0
        self.cur_data_right = 0
        self.cur_data_left = 0
        self.displacement_left = 0
        self.displacement_right = 0
        self.diff_left = 0
        self.diff_right = 0
        self.HeadingFrontBackFlg = 0 # 차량 앞뒤 방향 플래그 , offset업데이트시 사용

        self.gps_flag = 0
        self.e_DR = None
        self.n_DR = None
        # self.e_DR = 30.40
        # self.n_DR = 36.72

        self.e_DR = 0
        self.n_DR = 0
        self.position_flag = 0
        self.offset = 0

        self.t_Time_call = 0
        self.t_GPS_call = 0
        self.t_IMU_call = 0

        self.heading = 0
        self.gps_out = 0
        self.imu_out = 0
        self.headingAcc = 99999999
        self.e_filter = None
        self.n_filter = None

        self.cur_point = Point32()
        self.cur_points = PointCloud()
        self.paths = PointCloud()

        self.pub_c = rospy.Publisher('/cur_xy',PointCloud,queue_size = 1)
        self.pub_p = rospy.Publisher('/path',PointCloud,queue_size = 1)

        self.hAcc = 0
        self.count = 0

        #plot
        self.X1 = []
        self.Y1 = []
        self.X2 = []
        self.Y2 = []
        self.X3 = []
        self.Y3 = []

        self.filter_buffer = []

        self.mode = "mode_1"

    def mode_2(self):
        self.displacement_filter()
        self.DR()
        
        self.e_final = self.e_DR
        self.n_final = self.n_DR

        self.msg_write(self.msg)
        # print("I:",round(self.yaw_imu),"  g:",round(self.yaw_gps),"   O: ",round(self.offset),"  f:",round(self.yaw_final),"   gt",self.t_GPS_call,"  go",self.gps_out)

        self.pub.publish(self.msg)

    def mode_1(self):
        # print("im here")
        # for i in [1,2,3]: # 15 Hz
        for i in [1,2,3,4]: # 20 Hz
            if i == 1:
                # end = t.time() + 0.065 # 15 Hz
                end = t.time() + 0.05 # 20 Hz

                self.displacement_filter()
                self.DR()

                self.e_final = self.e_gps
                self.n_final = self.n_gps
                self.e_DR = self.e_gps
                self.n_DR = self.n_gps
                print()
                while (t.time() - end) < 0:
                    pass

            else:
                # end = t.time() + 0.065 # 15 Hz
                end = t.time() + 0.05  # 20 Hz

                self.yaw_final = self.yaw_imu + self.offset
                self.displacement_filter()
                self.DR()
                self.e_final = self.e_DR
                self.n_final = self.n_DR
                
                if i == 2 or i == 3 or i == 4:
                    while (t.time() - end) < 0:
                        pass
                else:
                    pass


        self.X1.append(self.e_gps)
        self.Y1.append(self.n_gps)
        self.X2.append(self.e_DR)
        self.Y2.append(self.n_DR)
        self.ps(self.X1,self.Y1,self.X2,self.Y2)   
        print("GPS : ",self.e_gps , "Dead Reckoning : ", self.e_DR)

        self.msg_write(self.msg)
        # print("I:",round(self.yaw_imu),"  g:",round(self.yaw_gps),"   O: ",round(self.offset),"  f:",round(self.yaw_final),"   gt",self.t_GPS_call,"  go",self.gps_out)

        self.pub.publish(self.msg)

    def msg_write(self,msg):
        self.msg.pose.pose.position.x = float(self.e_final)
        self.msg.pose.pose.position.y = float(self.n_final)
        self.msg.pose.pose.position.z = float(self.velocity)
        self.msg.twist.twist.angular.z = self.yaw_final
        self.e_final_pre = self.e_final
        self.n_final_pre = self.n_final

    def GPS_call(self,data):
        self.t_GPS_call = t.time()

        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z

        self.status = data.status.status #RTK status

        self.e_gps,self.n_gps,self.u_gps = pm.geodetic2enu(lat,lon,alt,base_lat,base_lon,base_alt)

        # self.mode_1()


    def GPS_Heading(self, data):
        self.yaw_gps = (450-(data.heading * 10**(-5)))%360
        self.hAcc = data.hAcc
        self.headingAcc =data.headAcc


    def IMU_call(self,data):
        # quaternion = (data.orientation.x,
        #     data.orientation.y,
        #     data.orientation.z,
        #     data.orientation.w)

        # euler = tf.transformations.euler_from_quaternion(quaternion)

        # pitch = euler[0] #x
        # roll = euler[1] #y
        # yaw_raw = euler[2]

        # self.yaw_imu = (math.degrees(euler[2]) + 90) % 360 # EAST = 0, ~360, ccw

        self.t_IMU_call = t.time()        
        self.yaw_imu = -data.orientation.x
        self.yaw_imu = self.yaw_imu%360
    
    # 왼쪽 엔코더
    def displacement_filter(self):
        # self.displacement_right = self.cur_data_right
        if self.encoder_flag == 0:
            self.displacement_left = self.cur_data_left
            self.encoder_flag = 1

        if (abs(self.cur_data_left - self.displacement_left) > 100):
            self.displacement_left = self.displacement_left + self.diff_left
        else:
            self.diff_left = self.cur_data_left - self.displacement_left
            self.displacement_left = self.cur_data_left

    # # 양쪽 엔코더
    # def displacement_filter(self):
    #     # self.displacement_right = self.cur_data_right
    #     if self.encoder_flag == 0:
    #         self.displacement_left = self.cur_data_left
    #         self.displacement_right = self.cur_data_right
    #         self.encoder_flag = 1

    #     if (abs(self.cur_data_left - self.displacement_left) > 100):
    #         self.displacement_left = self.displacement_left + self.diff_left
    #     else:
    #         self.diff_left = self.cur_data_left - self.displacement_left
    #         self.displacement_left = self.cur_data_left

        
    #     if (abs(self.cur_data_right - self.displacement_right) > 100):
    #         self.displacement_right = self.displacement_right + self.diff_right
    #     else:
    #         self.diff_right = self.cur_data_right - self.displacement_right            # 여기서 displacement_right는 이전값을 의미한다
    #         self.displacement_right = self.cur_data_right                              # 여기서 displacement_right는 현재값을 의미한다

    # #왼쪽 엔코더 콜백함수
    def Get_Dis_left(self, msg):
        self.cur_data_left = msg.encoder       
        print(msg.encoder)

    #오른쪽 엔코더 콜백함수
    def Get_Dis_right(self, data):
        res = data.data
        self.cur_data_right = int(res)


    def DR(self):
        if (self.dis_DR_flag == 0) and (self.displacement_left !=0):
            self.c = self.displacement_left
            self.d = self.displacement_left
            self.dis_DR_flag = 1
            self.e_DR = self.e_gps
            self.n_DR = self.n_gps
  
        elif self.dis_DR_flag == 1 :
            self.c = self.d
            self.d = self.displacement_left 

        if ((self.d - self.c) < -10000000):
            self.dis_DR_enc_left = (self.d + (256**4 - self.c))*1.6564/100 # encoder pulse를 m단위로 치환
        else:
            self.dis_DR_enc_left = (self.d - self.c)*1.6564/100

        if ((self.d - self.c)>0):
            self.HeadingFrontBackFlg = 1               
        else:
            self.HeadingFrontBackFlg = 0
            
        self.dis_DR_enc = self.dis_DR_enc_left # encoder 왼쪽 오른쪽 평균내기

        self.velocity = self.dis_DR_enc

        self.e_DR += self.dis_DR_enc*math.cos(math.radians(self.yaw_final))
        self.n_DR += self.dis_DR_enc*math.sin(math.radians(self.yaw_final))


    #Dead Reckoning 양쪽 엔코더
    # def DR(self):
    #     if (self.dis_DR_flag == 0) and (self.displacement_left !=0):
    #         self.c = self.displacement_left
    #         self.d = self.displacement_left
    #         self.e = self.displacement_right
    #         self.f = self.displacement_right
    #         self.dis_DR_flag = 1
        
    #     elif self.dis_DR_flag == 1 :
    #         self.c = self.d
    #         self.d = self.displacement_left 

    #         self.e = self.f
    #         self.f = self.displacement_right         

    #     if ((self.d - self.c) < -10000000):
    #         self.dis_DR_enc_left = (self.d + (256**4 - self.c))*1.6564/100 # encoder pulse를 m단위로 치환
    #     else:
    #         self.dis_DR_enc_left = (self.d - self.c)*1.6564/100

    #     if ((self.f - self.e) < -10000000):
    #         self.dis_DR_enc_right = (self.f + (256**4 - self.e))*1.6564/100 
    #     else:
    #         self.dis_DR_enc_right = (self.f - self.e)*1.6564/100
        
    #     if ((self.f - self.e)>0):
    #         self.HeadingFrontBackFlg = 1               
    #     else:
    #         self.HeadingFrontBackFlg = 0

    #     self.dis_DR_enc = (self.dis_DR_enc_right + self.dis_DR_enc_left)/2 # encoder 왼쪽 오른쪽 평균내기

    #     self.velocity = self.dis_DR_enc

    #     self.e_DR += self.dis_DR_enc*math.cos(math.radians(self.yaw_final))
    #     self.n_DR += self.dis_DR_enc*math.sin(math.radians(self.yaw_final))

    def yaw_check(self): # 차량이 똑바로 직진시, heading 정확도가 200000보다 낮을 시 heading값 보정
        if ((self.dis_DR_enc_right - self.dis_DR_enc_left) < 0.03) and self.HeadingFrontBackFlg == 1 and self.headingAcc < 200000:
            self.yaw_flag = 1               

        else:
            self.yaw_flag = 0

    def decide_heading(self):
        self.yaw_check()
        if self.imu_out == 0:               # imu 정상
            if self.gps_out == 0:           # gps 정상
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

                # print("I:",round(self.yaw_imu),"  g:",round(self.yaw_gps),"   O: ",round(self.offset),"  f:",round(self.yaw_final),"   gt",self.t_GPS_call,"  go",self.gps_out)                                                          # imu / gps가 꺼졌을 때
            else:  
                pass

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
        self.decide_heading()

        if self.t_Time_call - self.t_IMU_call > 0.2:
            self.imu_out = 1
        else:
            self.imu_out = 0

        self.t_Time_call = t.time()
        if self.t_Time_call - self.t_GPS_call < 0.2:
            self.mode = "mode_1"
            self.gps_out = 0
        else:
            self.mode = "mode_2"
            self.gps_out = 1

        self.mode = "mode_2"
        if self.mode == "mode_1":
            pass

        if self.mode == "mode_2":
            self.mode_2()

        self.X1.append(self.e_gps)
        self.Y1.append(self.n_gps) 
        self.X2.append(self.e_DR)
        self.Y2.append(self.n_DR)
        self.ps(self.X1,self.Y1,self.X2,self.Y2)   
        # print("GPS : ",self.e_gps , "Dead Reckoning : ", self.e_DR)
        

loc = Localization()
rospy.Subscriber('/serial', Serial_Info , loc.Get_Dis_left)
rospy.Subscriber("/Displacement_right", Int64, loc.Get_Dis_right)
rospy.Subscriber('/gps_data/fix',NavSatFix,loc.GPS_call)
rospy.Subscriber('/gps_data/navpvt',NavPVT, loc.GPS_Heading)
rospy.Subscriber('/imu',Imu,loc.IMU_call)
rospy.Subscriber("/timer",Time, loc.Time_call)
rate = rospy.Rate(500)
rospy.spin()
