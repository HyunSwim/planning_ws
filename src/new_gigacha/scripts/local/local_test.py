#!/usr/bin/env python3
#-*- coding:utf-8 -*-
# date : 0703

from __future__ import division, print_function #파이썬3문법을 2에서도 쓸수있게해줌
import serial, math, time, rospy
import numpy as np
import pymap3d as pm
import matplotlib.pyplot as plt

from sensor_msgs.msg import NavSatFix, Imu, PointCloud
from geometry_msgs.msg import Point32, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from ublox_msgs.msg import NavPVT
from new_gigacha.msg import Local, Serial_Info
from std_msgs.msg import Int64,Float32

from numpy.random import randn
from scipy.linalg import block_diag
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from filterpy.stats import plot_covariance_ellipse

PI = math.pi
base_lat = 37.3851693
base_lon = 126.6562271
base_alt = 15.4

class Localization():
    def __init__(self):
        rospy.init_node('Position_Node', anonymous=False)
        self.pub = rospy.Publisher('/pose', Local, queue_size = 1)

        self.msg = Local()
        self.serial_msg = Serial_Info()


        self.i_enc = []
        self.n_enc = []
        self.cnt = 0 

        self.yaw_gps = 0
        self.yaw_imu = 0

        self.transitional_velocity = 0
        self.rotational_velocity = 0

        self.t_old = time.time()
        self.t_new = time.time()
        self.t_delta = self.t_new - self.t_old

        self.radius_wheel = 0.25
        self.distance_btw_wheel = 0.97
        self.dis = 0
        
        # self.cur_point = Point32()
        # self.cur_points = PointCloud()
        # self.paths = PointCloud()

        # self.pub_c = rospy.Publisher('/cur_xy',PointCloud,queue_size = 1)
        # self.pub_p = rospy.Publisher('/path',PointCloud,queue_size = 1)

        self.hAcc = 0

        self.vel = TwistWithCovarianceStamped()
        self.vel.header.frame_id = "gps"

        self.pulse_right = 0
        self.pulse_left = 0

        self.e_gps = 0
        self.n_gps = 0
        self.u_gps = 0

        self.X1 = []
        self.Y1 = []
        self.X2 = []
        self.Y2 = []

        


    def all_filter(self):
        f = ExtendedKalmanFilter(dim_x=5,dim_z=5)
        dt = 0.1
        
        # State Transition Matrix F
        f.F = np.array([[1,0,-self.dis*math.sin(self.yaw_gps*PI/180),math.cos(self.yaw_gps*PI/180),0],
                        [0,1,self.dis*math.cos(self.yaw_gps*PI/180),math.sin(self.yaw_gps*PI/180),0],
                        [0,0,1,0,dt],
                        [0,0,0,1,0],
                        [0,0,0,0,1]])
                   
        
        # Noise (l)
        # q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.001)
        q1 = Q_discrete_white_noise(dim=2, dt=0.1, var=0.001)
        q2 = Q_discrete_white_noise(dim=3, dt=0.1, var=0.001)                  # moise Q 상태변수에 영향을 주는 잡음
        f.Q = block_diag(q1, q2)

        # Observe Matrix H
        # f.H = np.array([[1,0,0],
        #                 [0,1,0],
        #                 [0,0,1])
        # Obeservation Noise R (v)
        cov_x_enc = 0.01#0.0031360000000000003
        cov_y_enc = 0.01#0.0031360000000000003

        cov_x_gps = 0.01 #0.0007199025610000001
        cov_y_gps = 0.01 #0.0007199025610000001
        cov_y_yaw = 0.01
        cov_y_velocity = 0.01

        f.R = np.array([[self.cov_x_gps, self.cov_xy_gps, 0,0,0],                                 # 측정값의 잡음 vk의 행렬
                    [self.cov_xy_gps, self.cov_y_gps, 0,0,0],
                    [0, 0, 0.01,0,0],
                    [0, 0, 0,0.01,0],
                    [0, 0, 0,0,0.01]])  

        #######INITIAL CONDITIONS########
        f.x = np.array([[0, 0, 0, 0, 0]]).T
        f.P = np.eye(5) * 500.
        
        return f
        
    def Get_Dis_right(self,data):
        res = data.data
        self.pulse_right = int(res)/100 # rotate rate _ right

        return self.pulse_right

    def Get_Dis_left(self,data):
        self.n_enc = data.encoder
        if self.cnt == 0:
            self.i_enc = self.n_enc
            self.cnt = 1
        
        self.pulse_left = 256*(self.n_enc[1]-self.i_enc[1])+(self.n_enc[0]-self.i_enc[0])+\
            256**2*(self.n_enc[2]-self.i_enc[2])+256**3*(self.n_enc[3]-self.i_enc[3])

        return self.pulse_left/100 # rotate rate _ left

    def Get_Enc_vel(self):
        self.t_old = self.t_new
        self.t_new = time.time()
        self.t_delta = float(self.t_new - self.t_old)

        self.transitional_velocity = self.radius_wheel * (self.pulse_right + self.pulse_left) / (2 * self.t_delta)
        self.rotational_velocity = self.radius_wheel * (self.pulse_right - self.pulse_left) / (2 * self.distance_btw_wheel)

        self.dis = self.transitional_velocity*self.t_delta
        # self.theta =  self.rotational_velocity*self.t_delta
        # # print("theta=", self.theta)

        # ### x, y 좌표값 계산
        # if self.rotational_velocity==0:
        #     ### 회전 속도가 0일 때, runge-kutta integration
        #     # print("w=0")
        #     x =  self.transitional_velocity*self.t_delta*math.cos(self.theta + (self.rotational_velocity*self.t_delta)/2)
        #     y = self.transitional_velocity*self.t_delta*math.sin(self.theta + (self.rotational_velocity*self.t_delta)/2)
        # else:
        #     ### 회전 속도가 0이 아닐 때, exact integration
        #     # print("w!=0")
        #     ### 엔코더 버전
        #     x =  (self.transitional_velocity/self.rotational_velocity) * (math.sin(self.routes[-1][3]) - math.sin(self.theta))
        #     y =  - (self.transitional_velocity/self.rotational_velocity) * (math.cos(self.routes[-1][3]) - math.cos(self.theta))

        # self.dis_x , self.dis_y = x, y
    
    def fix_velocity(self,data):
        self.Get_Enc_vel()
        # print(1)
        vel_x = data.twist.twist.linear.x
        vel_y = data.twist.twist.linear.y

        gps_velocity = math.sqrt(vel_x**2 + vel_y**2)

        # print("gps = {}, encoder_tran = {}, encoder_rota = {}".format(gps_velocity, self.transitional_velocity, self.rotational_velocity))
        #print("gps_x = {0}"+"gps_y = {1}".format(vel_x,vel_y)

    def GPS_Heading(self, data):
        # self.yaw_gps = (450-(data.heading * 10**(-5)))%360 
        self.yaw_gps = (data.heading*10**(-5))%360  

        # print("gps_heading = {} , imu_heading = {}".format(self.yaw_gps,self.yaw_imu))

    # def HJacobian(self):

    #     return np.eye(5)

    # def Hx(self):
    #     return 1

        
    def GPS_call(self,data):

        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z

        # self.status = data.status.status #RTK status

        self.e_gps,self.n_gps,self.u_gps = pm.geodetic2enu(lat,lon,alt,base_lat,base_lon,base_alt)
        
        self.cov_x_gps = data.position_covariance[4]
        self.cov_y_gps = data.position_covariance[0]
        self.cov_xy_gps = data.position_covariance[1]

        #kalman filter 
        zs = [self.e_gps,self.n_gps,self.yaw_gps,self.dis,0]
        
        # a = filter.HJacobian()
        # self.HJacobian = np.eye(5)
        # self.Hx = 1
        def HJacobian(x):

            return np.eye(5)

        def Hx(x):
            return 1

        filter = self.all_filter()
        filter.predict()
        filter.update(zs,HJacobian,Hx)
        self.e_filter = filter.x[0]
        self.n_filter = filter.x[1]
        
        
        self.X1.append(self.e_gps)
        self.Y1.append(self.n_gps)

        self.X2.append(self.e_filter)
        self.Y2.append(self.n_filter)


        self.ps(self.X1, self.Y1, self.X2, self.Y2)

    def ps(self,X1,Y1,X2,Y2):
        plt.ion()
        animated_plot = plt.plot(X1, Y1, 'r')[0]
        animated_plot2 = plt.plot(X2, Y2, 'b')[0]
        ps1=1

        for i in range(0, len(X1)):
            if i>30:
                animated_plot.set_xdata(X1[0:i])
                animated_plot.set_ydata(Y1[0:i])
                animated_plot2.set_xdata(X2[0:i])
                animated_plot2.set_ydata(Y2[0:i])

            else:
                animated_plot.set_xdata(X1[0:i])
                animated_plot.set_ydata(Y1[0:i])

        plt.draw()
        plt.pause(0.01)    
    
    def IMU_call(self,data):       
        self.yaw_imu = data.orientation.x

        if self.yaw_imu < 0:
            # self.yaw_imu = self.yaw_imu + 360
            self.yaw_imu += 360

        self.yaw_imu = self.yaw_imu%360

loc = Localization()
rospy.Subscriber("/Displacement_right", Int64, loc.Get_Dis_right)
rospy.Subscriber("/serial", Serial_Info, loc.Get_Dis_left)
rospy.Subscriber('/ublox_gps/fix',NavSatFix,loc.GPS_call)
rospy.Subscriber('/ublox_gps/navpvt',NavPVT, loc.GPS_Heading)
rospy.Subscriber("/ublox_gps/fix_velocity", TwistWithCovarianceStamped, loc.fix_velocity)
rospy.Subscriber('/imu',Imu,loc.IMU_call)
rospy.spin()