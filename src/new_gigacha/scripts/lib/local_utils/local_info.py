from lib.local_utils.gps import GPS
from lib.local_utils.imu import IMU
from new_gigacha.msg import Position_Info

import rospy

class Localization_Info:
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()
        
        self.msg = Position_Info()

