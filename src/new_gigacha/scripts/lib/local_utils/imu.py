import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class IMU:
    def __init__(self):
        rospy.init_node("/Imu_node",anonymous = True)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def imu_call_back(self, data):
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

        return self.yaw

rospy.Subscriber('/imu',Imu,IMU.imu_call_back)
rospy.spin()