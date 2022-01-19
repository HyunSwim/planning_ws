import rospy
from sensor_msgs.msg import NavSatFix
import pymap3d as pm

class GPS:
    def __init__(self):
        rospy.init_node("/Gnss_Node", anonymous = True)
        self.x = 0
        self.y = 0
        self.z = 0
    
    def gps_call_back(self,data):
        base = rospy.get_param("songdo_base")

        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z

        self.x, self.y, self.z = pm.geodetic2enu(lat,lon,alt,base[0],base[1],base[2])

        return self.x, self.y

rospy.Subscriber('/ublox_gps/fix',NavSatFix,GPS.gps_call_back)
rospy.spin()