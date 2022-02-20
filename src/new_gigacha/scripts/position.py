#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from new_gigacha.msg import Local
from ublox_msgs.msg import NavPVT

from lib.local_utils.gps import GPS
from lib.local_utils.imu import IMU


class Localization():
    def __init__(self):
        rospy.init_node('Localization', anonymous=False)
        self.pub = rospy.Publisher('/pose', Local, queue_size = 1)
        self.msg = Local()

        self.gps = GPS()
        self.imu = IMU()

    def main(self):
        self.msg.x = self.gps.x
        self.msg.y = self.gps.y
        self.msg.heading = self.imu.yaw

        self.pub.publish(self.msg)

if __name__ == '__main__':
    loc = Localization()
    rate = rospy.Rate(10)
 
    while not rospy.is_shutdown():
        loc.main()
        rate.sleep()
