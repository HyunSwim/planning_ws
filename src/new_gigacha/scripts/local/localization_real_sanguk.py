#!/usr/bin/env python3
from __future__ import division, print_function #파이썬3문법을 2에서도 쓸수있게해줌
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from new_gigacha.msg import Local
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Int64,Float32
# from tf.transformations import euler_from_quaternion
import pymap3d
from numpy import rad2deg
from ublox_msgs.msg import NavPVT
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
import numpy as np
import math
import matplotlib.pyplot as plt

PI = math.pi

#roslaunch ublox_gps ublox_zed-f9p.launch
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
        
        #Songdo
        self.lat_origin = 37.3851693 
        self.lon_origin = 126.6562271
        self.alt_origin = 15.4

        self.yaw_gps = 0
        self.hAcc = 0
        self.headingAcc = 0
        self.offset = 0
        self.yaw_imu = 0
        self.final_yaw = 0
        self.gear = None
        self.HeadingFrontBackFlg = 1
        self.HJacobian=np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        self.Hx=np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        self.X1 = []
        self.Y1 = []
        self.X2 = []
        self.Y2 = []
        self.encoder_flag = 0
        self.dis_gps_flag = 0
        self.dis_DR_flag = 0
        self.dis_DR_enc_left = 0
        self.dis_DR_enc_right = 0 
        self.dis_DR_enc = 0
        self.dis_gps_enc = 0
        self.velocity = 0

        self.displacement = 0
        self.cur_data_right = 0
        self.displacement_left = 0
        self.displacement_right = 0
        self.diff_left = 0
        self.diff_right = 0
        self.HeadingFrontBackFlg = 1 # 차량 앞뒤 방향 플래그 , offset업데이트시 사용

        self.c = 0
        self.d = 0
        self.e = 0
        self.f = 0
        


        rospy.Subscriber("/simul_gps", Pose, self.gpsCallback)
        rospy.Subscriber("/simul_imu", Pose, self.imuCallback)
        
        rospy.Subscriber('/ublox_gps/navpvt',NavPVT, self.gps_Heading)
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.gpsCallback)
        rospy.Subscriber("/imu", Imu, self.imuCallback)

        rospy.Subscriber('/Displacement', Int64 , self.Get_Dis_left)
        rospy.Subscriber("/Displacement_right", Int64, self.Get_Dis_right)



    def main(self):
        self.decide_heading()        
        self.pub.publish(self.msg)
        self.vis_msg.pose.position.x = self.msg.x
        self.vis_msg.pose.position.y = self.msg.y
        self.vis_msg.header.stamp = rospy.Time.now()
        self.vis_pub.publish(self.vis_msg)
        print("Localization is on...")

    def all_filter(self):
        f = KalmanFilter(dim_x=4,dim_z=4)
        dt = 0.1
        
        # State Transition Matrix F
        # f.F = np.array([[1,0,0,0],
        #                 [0,1,0,0],
        #                 [0,0,1,0],
        #                 [0,0,0,1]])
        # f.B = np.array([[math.cos(self.final_yaw)*dt,0],
        #                 [math.sin(self.final_yaw)*dt,0],
        #                 [0,0],
        #                 [0,0]])       
        
        f.F = np.array([[1,0,0,math.cos(self.final_yaw*PI/180)*dt],
                        [0,1,0,math.sin(self.final_yaw*PI/180)*dt],
                        [0,0,1,0],
                        [0,0,0,1]])
        
        # Noise (l)
        q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.001)                # moise Q 상태변수에 영향을 주는 잡음
        f.Q = block_diag(q, q)

        # Observe Matrix H
        f.H = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        # Obeservation Noise R (v)
        cov_x_enc = 0.01#0.0031360000000000003
        cov_y_enc = 0.01#0.0031360000000000003

        cov_x_gps = 0.01 #0.0007199025610000001
        cov_y_gps = 0.01 #0.0007199025610000001
        cov_y_yaw = 0.01
        cov_y_velocity = 0.01

        f.R = np.array([[self.cov_x_gps, self.cov_xy_gps, 0, 0],                                 # 측정값의 잡음 vk의 행렬
                    [self.cov_xy_gps, self.cov_y_gps, 0, 0],
                    [0, 0, cov_y_yaw, 0],
                    [0, 0, 0, cov_y_velocity]])  

        #######INITIAL CONDITIONS########
        f.x = np.array([[0, 0, 0, 0]]).T
        f.P = np.eye(4) * 500.
        
        return f

    def H_of(self,x):
        
        return np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]) 


    def gpsCallback(self, data):
        self.msg.x, self.msg.y, _ = pymap3d.geodetic2enu(data.latitude, data.longitude, self.alt_origin, \
                                            self.lat_origin , self.lon_origin, self.alt_origin)

        self.cov_x_gps = data.position_covariance[4]
        self.cov_y_gps = data.position_covariance[0]
        self.cov_xy_gps = data.position_covariance[1]

        #kalman filter 
        zs = (self.msg.x,self.msg.y,self.yaw_final,self.velocity)
        
        filter = self.all_filter()
        filter.predict()
        # filter.predict(np.array([1,0]))
        filter.update(zs)
        
        
        # filter.update(zs)
        self.e_filter = filter.x[0]
        self.n_filter = filter.x[1]

        self.X1.append(self.msg.x)
        self.Y1.append(self.msg.y)

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

    def Get_Dis_right(self, data):
        res = data.data
        self.cur_data_right = int(res)

    def Get_Dis_left(self, data):
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

        self.velocity = self.dis_DR_enc_right/0.1

    def imuCallback(self, data):
        self.vis_msg.pose.orientation = data.orientation
        self.yaw_imu = -data.orientation.x
        self.msg.heading = self.yaw_final

        # self.msg.heading = rad2deg(self.yaw) % 360 #East = 0, North = 90, West = 180, South = 270 deg
        
    def gps_Heading(self, data):
        self.yaw_gps = (450-(data.heading * 10**(-5)))%360
        self.hAcc = data.hAcc
        self.headingAcc =data.headAcc

    def yaw_check(self):
        if self.gear == None:
            self.gear = 0
            
        if self.HeadingFrontBackFlg == 1 and self.headingAcc < 700000 and self.gear == 0 :
            self.yaw_flag = 1               
        else:
            self.yaw_flag = 0

    def decide_heading(self):
        self.DR()
        self.yaw_check()                    
        if self.yaw_flag == 1: 
            self.offset = self.yaw_gps - self.yaw_imu 
            self.yaw_final = self.yaw_imu + self.offset
            self.yaw_final = self.yaw_final % 360

        elif self.yaw_flag == 0:
            self.yaw_final = self.yaw_imu % 360

        # self.yaw_final = self.yaw_imu % 360
        
         
            
if __name__ == '__main__':
    
    loc = Localization()
    rate = rospy.Rate(50)
 
    while not rospy.is_shutdown():

        loc.main()
        rate.sleep()