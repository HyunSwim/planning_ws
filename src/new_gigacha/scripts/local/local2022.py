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

# 좌표변환모듈
# import pymap3d as pm
from pyproj import Proj, transform
import pandas  as pd

import time as t
import threading
from std_msgs.msg import Int64,Float32

# define
PI = 3.141592

# ENU base coordinates
# 송도
base_lat = 37.383784
base_lon = 126.654310 
base_alt = 15.4


#utm-k
proj_WGS84 = Proj(init='epsg:4326') # Wgs84 경도/위도, GPS사용 전지구 좌표
proj_UTMK = Proj(init='epsg:5178') # UTM-K(Bassel) 도로명주소 지도 사용 중

class Localization():
    def __init__(self):
        rospy.init_node('Position_Node', anonymous=False)
        self.pub = rospy.Publisher('/pose', Odometry, queue_size = 1)
        self.msg = Odometry()


        self.e_gps = None
        self.n_gps = None


        self.status = 0 # gps -1 , 0, 1, 2

        self.yaw_gps = None
        self.yaw_imu = None
        self.yaw_final = None
        self.offset = 0

        self.diff_left = 0
        self.diff_right = 0
        self.HeadingFrontBackFlg = None # 차량 앞뒤 방향 플래그 , offset업데이트시 사용


        self.position_flag = 0
        self.yaw_flag = 0

        self.t_Time_call = 0 
        self.t_GPS_call = None
        self.t_IMU_call = 0

        self.heading = 0
        self.gps_out = 0
        self.imu_out = 0
        self.headingAcc = 18000000
        self.e_filter = None
        self.n_filter = None

        self.cur_point = Point32()
        self.cur_points = PointCloud()
        self.paths = PointCloud()


        self.hAcc = 0


        self.filter_buffer = []


    def gps_check(self): #gps신뢰도 판단하는 부분 
        if self.hAcc > 210: # rtk정상 연결시 14~25 
            self.position_flag = 1 # RTK 연결끊기면 
        else:
            self.position_flag = 0 # RTK 연결되면 

    def msg_write(self,msg): #좌표, 헤딩을 msg에 쓰는 함수 
        self.msg.pose.pose.position.x = float(self.e_gps)
        self.msg.pose.pose.position.y = float(self.n_gps)
        # self.msg.pose.pose.position.z = float(self.velocity) #실제 정확하지 못했던 속도
        self.msg.twist.twist.angular.z = self.yaw_final


    def GPS_fix_call(self,data): #GPS 패키지로부터 위경도 좌표 받아오는 콜백 함수
        self.t_GPS_call = t.time()

        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z
        self.status = data.status.status #RTK status

        # UTM-K좌표로 변환
        self.e_gps, self.n_gps = transform(proj_WGS84, proj_UTMK, lat, lon)

        #위도경도 좌표를 직가평면좌표계로 변환 (파이맵3디 상대좌표)
        #self.e_gps,self.n_gps,self.u_gps = pm.geodetic2enu(lat,lon,alt,base_lat,base_lon,base_alt) 

        self.gps_check()


    def GPS_navpvt_call(self, data):
        self.yaw_gps = (450-(data.heading * 10**(-5)))%360 # 사용하는 헤딩단위 맞춤
        self.hAcc = data.hAcc #수평정확도 좌표 정확도 
        self.headingAcc =data.headAcc

    def IMU_call(self,data):
        self.t_IMU_call = t.time()        
        self.yaw_imu = -data.orientation.x
        self.yaw_imu = self.yaw_imu%360

    def decide_heading(self): # heading보정 하는 로직 추가 하시면됩니다.
        self.yaw_final = self.yaw_imu # 지금은 단순히 imu를 사용


    def Time_call(self,time):
        self.decide_heading()
        self.msg_write(self.msg)
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg) 
        print("I:",round(self.yaw_imu),"  g:",round(self.yaw_gps),"  f:",round(self.yaw_final))
        print("gps hAcc: ",self.hAcc, "   e :",self.e_gps,"  n :",self.n_gps)

loc = Localization()
rospy.Subscriber('/gps_data/fix',NavSatFix,loc.GPS_fix_call)
rospy.Subscriber('/gps_data/navpvt',NavPVT, loc.GPS_navpvt_call)
rospy.Subscriber('/imu',Imu,loc.IMU_call)
rospy.Subscriber("/timer",Time, loc.Time_call)
rate = rospy.Rate(500)
rospy.spin()
