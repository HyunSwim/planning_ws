#!/usr/bin/env python
#-*- coding:utf-8 -*-
# date : 0703

from __future__ import division, print_function #파이썬3문법을 2에서도 쓸수있게해줌
import serial, math, time, rospy
import numpy as np
import pymap3d as pm

from sensor_msgs.msg import NavSatFix, Imu, PointCloud
from geometry_msgs.msg import Point32, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from ublox_msgs.msg import NavPVT
from new_gigacha.msg import Local, Serial_Info
from std_msgs.msg import Int64,Float32

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
        
        # self.cur_point = Point32()
        # self.cur_points = PointCloud()
        # self.paths = PointCloud()

        # self.pub_c = rospy.Publisher('/cur_xy',PointCloud,queue_size = 1)
        # self.pub_p = rospy.Publisher('/path',PointCloud,queue_size = 1)

        self.hAcc = 0

        self.vel = TwistWithCovarianceStamped()
        self.vel.header.frame_id = "gps"

    def Get_Dis_right(self,data):
        res = data.data
        self.pulse_right = int(res)/100 # rotate rate _ right

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
    
    def fix_velocity(self,data):
        # print(1)
        vel_x = data.twist.twist.linear.x
        vel_y = data.twist.twist.linear.y

        gps_velocity = math.sqrt(vel_x**2 + vel_y**2)

        print("gps = {0}"+"encoder_tran = {1}"+"encoder_rota = {2}".format(gps_velocity, self.transitional_velocity, self.rotational_velocity))
        #print("gps_x = {0}"+"gps_y = {1}".format(vel_x,vel_y)

    def GPS_Heading(self, data):
        # self.yaw_gps = (450-(data.heading * 10**(-5)))%360 
        self.yaw_gps = data.heading*10**(-5)     

        print("gps_heading = {0}"+"imu_heading = {1}".format(self.yaw_gps,self.yaw_imu))
    
    def IMU_call(self,data):       
        self.yaw_imu = -data.orientation.x
        self.yaw_imu = self.yaw_imu%360

loc = Localization()
rospy.Subscriber("/Displacement_right", Int64, loc.Get_Dis_right)
rospy.Subscirber("/serial", Serial_Info, loc.Get_Dis_left)
rospy.Subscriber('/ublox_gps/navpvt',NavPVT, loc.GPS_Heading)
rospy.Subscriber("/ublox_gps/fix_velocity", TwistWithCovarianceStamped, loc.fix_velocity)
rospy.Subscriber('/imu',Imu,loc.IMU_call)
rospy.spin()