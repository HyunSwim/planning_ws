#!/usr/bin/env python3
import rospy
from lib.euler_from_quaternion import euler_from_quaternion as efq

from sensor_msgs.msg import Imu


class IMU():
    def __init__(self):
        self.yaw = 0
        self.yaw_rate = 0

        rospy.Subscriber("/imu", Imu, self.imu_call_back)


    def imu_call_back(self, data):
        self.vis_msg.pose.orientation = data.orientation
        self.yaw_rate = -data.angular_velocity.z

        orientation_q = data.orientation
        roll, pitch, yaw = efq(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        self.yaw = np.rad2deg(yaw)

if __name__ == '__main__':
    try:
        imu=IMU()
    except rospy.ROSInterruptException:
        pass