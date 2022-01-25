#!/usr/bin/env python
#-*- coding:utf-8 -*-

import serial
import rospy
import base64
import struct
from math import radians
from math import degrees
from math import sin
from math import cos
from math import pi
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class imuencoder:
	def __init__(self):
		self.ser = serial.Serial('/dev/ttyUSB0', 115200)
		self.yaw_prev = 0
		self.yaw = 0
		self.yaw_diff = 0
		self.loop_init = 0
		self.tmp1 = 0
		self.tmp2 = 0
		self.tmp3 = 0
		self.tmp4 = 0
		self.tmp5 = 0
		self.n_enc = []
		self.i_enc = []
		self.thres = 0
		self.delta_x = 0
		self.delta_y = 0
		self.x = 0
		self.y = 0
		
		self.pub_xy = rospy.Publisher('/cur_xy', Odometry, queue_size=1)
		self.cur_xy = Odometry()
		self.get_encoder()
		self.i_enc = list(self.get_encoder())
		#print("initial: ")
		#print(self.i_enc)

	def imu_callback(self, data):
		self.yaw = data.orientation.x + 90
		if self.loop_init == 0:
			self.yaw_prev = self.yaw
			print(self.yaw_prev) ## CHECK!!
			self.loop_init = 1

	def get_encoder(self):
		result2 = self.ser.read_until(b'\x0d\x0a')
		result1 = result2[1:]
		result_remainning = result2[3:10]
		# print(type(result_remainning))
		if len(result1) == 22:
			header = result1[0:3].decode()
			# print(header)
			self.tmp1, self.tmp2, self.tmp3, self.tmp4, self.tmp5 = struct.unpack("BBBBB", result1[11:16])
			print(self.tmp1, self.tmp2, self.tmp3, self.tmp4, self.tmp5)
			return self.tmp1, self.tmp2

	def calc_yawDiff(self):
		self.yaw_diff = (self.yaw - self.yaw_prev)
		if(self.yaw_diff>300):
			#print("do1?")
			self.yaw_diff = (self.yaw-self.yaw_prev-360)
		elif(self.yaw_diff<-300):
			#print("do2?")
			self.yaw_diff = (self.yaw-self.yaw_prev+360)

	def calc_Coordi(self):
		self.delta_x = 0.05*sin(pi * (self.yaw_prev/180.0))
		self.delta_y = 0.05*cos(pi * (self.yaw_prev/180.0))
		self.x = round(self.x + self.delta_x,5)
		self.y = round(self.y + self.delta_y,5)
		#print("deltax:", self.delta_x)
		#print("deltay:", self.delta_y)
		# print(self.x)
		# print(self.y)
		# print(self.yaw_prev)
		self.cur_xy.pose.pose.position.x = self.x
		self.cur_xy.pose.pose.position.y = self.y


	def main(self):
		rospy.Subscriber("/imu", Imu, self.imu_callback)
		self.n_enc = list(self.get_encoder())
		#print(self.i_enc)
		#print(self.n_enc)
		self.thres = 256*(self.n_enc[1]-self.i_enc[1])+(self.n_enc[0]-self.i_enc[0])
		#print(self.thres)
		#print(self.yaw_prev)
		#print(self.yaw)
		# print("thres: ", self.thres)
		# print(self.i_enc)
		# print(self.n_enc)
		if self.thres > 256 or self.thres < 0:
			self.calc_yawDiff()
			self.yaw_prev = self.yaw
			self.calc_Coordi()
			#print("yaw_prev : ", self.yaw_prev)
			#print("yaw: ", self.yaw)
			self.i_enc = self.n_enc
		
		self.pub_xy.publish(self.cur_xy)
	
if __name__=="__main__":
	rospy.init_node('encoder', anonymous = False)

	enc = imuencoder()

	while not rospy.is_shutdown():
		enc.main()

	#rospy.spin()