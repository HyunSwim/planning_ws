import serial
from std_msgs.msg import Int64
import rospy
import signal
import sys

def signal_handler(sig,frame):
    print('Shut Down')
    sys.exit(0)

class Localization():
	def __init__(self):
		rospy.init_node('Displacement_right', anonymous=False)
		self.ser = serial.Serial(port = '/dev/ttyACM1',
		baudrate = 115200,
		)
		self.pub = rospy.Publisher('/Displacement_right', Int64, queue_size = 1)
		signal.signal(signal.SIGINT,signal_handler)

	def main(self):

		res = self.ser.readline()
		while True:
			try:
				print(int(res))
				break
			except:
				res = self.ser.readline()

		self.pub.publish(int(res))

loc = Localization()
while not rospy.is_shutdown():
	loc.main()