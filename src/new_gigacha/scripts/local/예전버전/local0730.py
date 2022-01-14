#!/usr/bin/env python
#-*- coding:utf-8 -*-
# date : 0730
# 시리얼 노드가 바뀌면서 데드레코닝 sync가 뒤틀려버림 이걸 수정해야함.
# 어떻게 할거냐. GPS rate도 중가가 필요하다고 판단. (기존 GPS Hz 5) > DR을 통한 20Hz로 헤르쯔를 올릴거임
# time.py의 속도를 20hz로 증가시킴. 거기에서 데드레코닝을 꾸준히 돌림
# left랑 right의 sync가 잘 안맞기 때문에 20hz를 주는 time.py로 sync를 맞춰줌.
# GPS의 신호 사이에 3개의 DR이 들어감.
# time, GPS, sync를 맞추기는 어려움, 주기적인 GPS를 가지고 SYnc를 맞추자
# 수정해야 할 것 : velocity 추가 및 확인하기, count확인하기

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
    

class Localization():
    def __init__(self):
        rospy.init_node('Position_Node', anonymous=False)
        self.pub = rospy.Publisher('/pose', Odometry, queue_size = 1)

        self.msg = Odometry()


        self.e_gps = None
        self.n_gps = None
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
        self.yaw_final = None

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
        self.t_GPS_call = None
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


    def mode_1(self):
        for i in [1,2,3,4]:
            if i == 1:
                end = time.time() + 0.05
                
                self.offset = self.yaw_gps - self.yaw_imu 
                self.yaw_final = self.yaw_imu + self.offset
                self.e_final = self.e_gps
                self.n_final = self.n_gps
                self.e_DR = self.e_gps
                self.n_DR = self.n_gps
                
                while (time.time() - end) > 0:
                    pass

            else:
                end = time.time() + 0.05

                self.yaw_final = self.yaw_imu + self.offset
                self.displacment_filter()
                self.DR()
                self.e_final = self.e_DR
                self.n_final = self.n_DR
                
                if i == 2 or i == 3:
                    while (time.time() - end) > 0:
                        pass
                else:
                    pass

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

        self.mode_1()
        
    def IMU_call(self,data):
        self.t_IMU_call = t.time()        
        self.yaw_imu = -data.orientation.x
        self.yaw_imu = self.yaw_imu%360
        
    def displacement_filter(self):
        self.displacement_right = self.cur_data_right
        if self.encoder_flag == 0:
            self.displacement_left = self.cur_data_left
            self.displacement_right = self.cur_data_right
            self.encoder_flag = 1

        if (abs(self.cur_data_left - self.displacement_left) > 100):
            self.displacement_left = self.displacement_left + self.diff_left
        else:
            self.diff_left = self.cur_data_left - self.displacement_left
            self.displacement_left = self.cur_data_left

        
        if (abs(self.cur_data_right - self.displacement_right) > 100):
            self.displacement_right = self.displacement_right + self.diff_right
        else:
            self.diff_right = self.cur_data_right - self.displacement_right            # 여기서 displacement_right는 이전값을 의미한다
            self.displacement_right = self.cur_data_right                              # 여기서 displacement_right는 현재값을 의미한다

    #왼쪽 엔코드 콜백함수
    def Get_Dis_left(self, msg):
        print(self.cur_data_right)
        self.cur_data_left = msg.encoder       

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
        
        elif self.dis_DR_flag == 1 :
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
        
        if ((self.f - self.e)>0):
            self.HeadingFrontBackFlg = 1               

        else:
            self.HeadingFrontBackFlg = 0

        self.dis_DR_enc = (self.dis_DR_enc_right + self.dis_DR_enc_left)/2 # encoder 왼쪽 오른쪽 평균내기

        self.e_DR += self.dis_DR_enc*math.cos(self.yaw_final*3.141592/180)
        self.n_DR += self.dis_DR_enc*math.sin(self.yaw_final*3.141592/180)

    def decide_heading(self):
        self.offset = self.yaw_gps - self.yaw_imu 
        self.yaw_final = self.yaw_imu + self.offset

    def ps(self,X1,Y1,X2,Y2,X3,Y3):
        plt.ion()
        animated_plot = plt.plot(X1, Y1, 'r')[0]
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

loc = Localization()
rospy.Subscriber('/serial', Serial_Info , loc.Get_Dis_left)
rospy.Subscriber("/Displacement_right", Int64, loc.Get_Dis_right)
rospy.Subscriber('/gps_data/fix',NavSatFix,loc.GPS_call)
rospy.Subscriber('/gps_data/navpvt',NavPVT, loc.GPS_Heading)
rospy.Subscriber('/imu',Imu,loc.IMU_call)
rospy.Subscriber("/timer",Time, loc.Time_call)
rate = rospy.Rate(500)
rospy.spin()

#!/usr/bin/env python
#-*- coding:utf-8 -*-
# date : 0730
# 시리얼 노드가 바뀌면서 데드레코닝 sync가 뒤틀려버림 이걸 수정해야함.
# 어떻게 할거냐. GPS rate도 중가가 필요하다고 판단. (기존 GPS Hz 5) > DR을 통한 20Hz로 헤르쯔를 올릴거임
# time.py의 속도를 20hz로 증가시킴. 거기에서 데드레코닝을 꾸준히 돌림
# left랑 right의 sync가 잘 안맞기 때문에 20hz를 주는 time.py로 sync를 맞춰줌.
# GPS의 신호 사이에 3개의 DR이 들어감.
# time, GPS, sync를 맞추기는 어려움, 주기적인 GPS를 가지고 SYnc를 맞추자
# 수정해야 할 것 : velocity 추가 및 확인하기, count확인하기

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
    

class Localization():
    def __init__(self):
        rospy.init_node('Position_Node', anonymous=False)
        self.pub = rospy.Publisher('/pose', Odometry, queue_size = 1)

        self.msg = Odometry()


        self.e_gps = None
        self.n_gps = None
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
        self.yaw_final = None

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
        self.t_GPS_call = None
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


    def mode_1(self):
        if self.coount == 1:
            self.offset = self.yaw_gps - self.yaw_imu 
            self.yaw_final = self.yaw_imu + self.offset
            self.e_final = self.e_gps
            self.n_final = self.n_gps
            self.e_DR = self.e_gps
            self.n_DR = self.n_gps
        else:
            self.yaw_final = self.yaw_imu + self.offset
            self.displacment_filter()
            self.DR()
            self.e_final = self.e_DR
            self.n_final = self.n_DR

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

        self.mode_1(self)
        
    def IMU_call(self,data):
        self.t_IMU_call = t.time()        
        self.yaw_imu = -data.orientation.x
        self.yaw_imu = self.yaw_imu%360
        
    def displacement_filter(self):
        self.displacement_right = self.cur_data_right
        if self.encoder_flag == 0:
            self.displacement_left = self.cur_data_left
            self.displacement_right = self.cur_data_right
            self.encoder_flag = 1

        if (abs(self.cur_data_left - self.displacement_left) > 100):
            self.displacement_left = self.displacement_left + self.diff_left
        else:
            self.diff_left = self.cur_data_left - self.displacement_left
            self.displacement_left = self.cur_data_left

        
        if (abs(self.cur_data_right - self.displacement_right) > 100):
            self.displacement_right = self.displacement_right + self.diff_right
        else:
            self.diff_right = self.cur_data_right - self.displacement_right            # 여기서 displacement_right는 이전값을 의미한다
            self.displacement_right = self.cur_data_right                              # 여기서 displacement_right는 현재값을 의미한다

    #왼쪽 엔코드 콜백함수
    def Get_Dis_left(self, msg):
        print(self.cur_data_right)
        self.cur_data_left = msg.encoder       

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
        
        elif self.dis_DR_flag == 1 :
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
        
        if ((self.f - self.e)>0):
            self.HeadingFrontBackFlg = 1               

        else:
            self.HeadingFrontBackFlg = 0

        self.dis_DR_enc = (self.dis_DR_enc_right + self.dis_DR_enc_left)/2 # encoder 왼쪽 오른쪽 평균내기

        self.e_DR += self.dis_DR_enc*math.cos(self.yaw_final*3.141592/180)
        self.n_DR += self.dis_DR_enc*math.sin(self.yaw_final*3.141592/180)

    def decide_heading(self):
        self.offset = self.yaw_gps - self.yaw_imu 
        self.yaw_final = self.yaw_imu + self.offset

    def ps(self,X1,Y1,X2,Y2,X3,Y3):
        plt.ion()
        animated_plot = plt.plot(X1, Y1, 'r')[0]
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

loc = Localization()
rospy.Subscriber('/serial', Serial_Info , loc.Get_Dis_left)
rospy.Subscriber("/Displacement_right", Int64, loc.Get_Dis_right)
rospy.Subscriber('/gps_data/fix',NavSatFix,loc.GPS_call)
rospy.Subscriber('/gps_data/navpvt',NavPVT, loc.GPS_Heading)
rospy.Subscriber('/imu',Imu,loc.IMU_call)
rospy.Subscriber("/timer",Time, loc.Time_call)
rate = rospy.Rate(500)
rospy.spin()

