from lib.local_utils.imu import IMU
from new_gigacha.msg import Position_Info
from ublox_msgs.msg import NavPVT
from std_msgs.msg import Int64

import rospy

class Localization_Info:
    def __init__(self):
        rospy.init_node("Position_Info", anonymous=False)
        self.pub = rospy.Publisher("/position", Position_Info, queue_size=1)

        self.imu = IMU()

        # from gps
        self.hAcc = 0
        self.headingAcc = 0
        self.gps_yaw = 0
        
        # msg
        self.msg = Position_Info()
        self.msg.imu_heading = self.imu.yaw

    def gps_heading(self, data):
        self.hAcc = data.hAcc
        self.gps_yaw = (450-(data.heading * 10**(-5)))%360

        self.msg.gps_heading = self.gps_yaw

        if self.hAcc > 210:
            self.msg.gps_flag = "ON"
        else:
            self.msg.gps_flag = "OFF"
    
    def decide_direction(self):
        enc_pre = self.encoder_right()
        enc = self.encoder_right()

        enc_diff = enc - enc_pre

        if enc_diff > 0:
            self.msg.direction = "Foward"
        elif enc_diff == 0:
            self.msg.direction = "Stop"
        else:
            self.msg.direction = "Backward"


    def encoder_right(self, data):
        return data.data

    def run(self):
        self.pub.publish(self.msg)


if __name__ == "__main__":
    loc = Localization_Info()
    rospy.Subscriber('/ublox_gps/navpvt',NavPVT, loc.gps_heading)
    rospy.Subscriber("/Displacement_right", Int64, loc.encoder_right)

    rate = rospy.Rate(20)

    while not rospy.is_shutdwon():
        loc.run()
        rate.sleep()