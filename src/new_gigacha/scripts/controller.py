from lib.general_utils.sig_int_handler import Activate_Signal_Interrupt_Handler
from lib.general_utils.read_global_path import read_global_path
from lib.controller_utils.pure_pursuit import PurePursuit
from lib.controller_utils.state import State
from lib.controller_utils.stanley import Stanley_Method
from lib.controller_utils.lateral_combined import Combined_Method
from lib.controller_utils.state_updater import stateUpdater
from lib.controller_utils.path_updater import pathUpdater
from lib.controller_utils.longtidudinal_controller import longitudinalController
from lib.controller_utils.pid_new_control import PID_C
from new_gigacha.msg import Local, Path, Planning_Info, Control_Info
import numpy as np

import rospy

class Controller:
    def __init__(self):

        print(f"Controller: Initializing Controller...")
        rospy.init_node("Controller", anonymous=False)
        self.control_pub = rospy.Publisher("/controller", Control_Info, queue_size=1)
        self.control_msg = Control_Info()

        self.state = State()
        self.global_path = read_global_path('songdo', 'global_simul')
        self.parking_path = read_global_path('songdo', 'parking_only_simul')
        # self.local_path = Path()
        
        self.update_state = stateUpdater(self.state)
        # self.update_local_path = pathUpdater(self.local_path)

<<<<<<< HEAD
        self.state.target_speed = 5.0 #TODO: decided by mission or map

        self.lat_controller= PurePursuit(self.state, self.global_path, self.local_path) 
        # self.lat_controller= Stanley_Method(self.state, self.global_path, self.local_path)
        # self.lat_controller= Combined_Method(self.state, self.global_path, self.local_path)
>>>>>>> PID_Auto_tuning
        # self.curve_check = min(max (self.lat_controller.deaccel(), -27), 27)
        self.lon_controller = longitudinalController(self.state)

    def run(self):
        # self.lat_controller.make_yaw()                 #stanley
        if self.state.mode == "emergency_stop":            
=======
        if self.state.mode == "emergency_stop":
>>>>>>> PID_Auto_tuning
            self.publish_control_info(1, 2)

        elif self.state.mode == "stop":
            self.publish_control_info(1, 0)

        elif self.state.mode == "backward":
            self.publish_control_info(0, 2)
<<<<<<< HEAD
            self.state.target_speed = 2.0           
        
        elif self.state.mode == "parking_driving":
            self.publish_control_info(0, 0)
            self.state.target_speed = 2.0 
=======
            self.state.target_speed = 5.0
        
        elif self.state.mode == "parking_driving":
            self.publish_control_info(0, 0)
            self.state.target_speed = 5.0
>>>>>>> PID_Auto_tuning
            
        else:
            self.publish_control_info(0, 0)
            self.state.target_speed = 5.0
            # if self.curve_check > 20 :
            #     self.state.target_speed = 10.0 - abs(self.curve_check)/10
            # else :
            #     self.state.target_speed = 5.0

        print(self.control_msg)

    def publish_control_info(self, estop, gear):
            self.control_msg.emergency_stop = estop
            self.control_msg.gear = gear
            self.control_msg.steer = self.lat_controller.run()
<<<<<<< HEAD
            self.control_msg.speed, self.control_msg.brake = self.lon_controller.run()   
=======
            print(f"steer : {self.control_msg.steer}")
            ####################For PID Tuining
            # self.control_msg.steer = 0 
            #######################################
            self.control_msg.speed, self.control_msg.brake = self.lon_controller.run()
>>>>>>> PID_Auto_tuning
            self.control_pub.publish(self.control_msg)
    
if __name__ == "__main__":
    
    Activate_Signal_Interrupt_Handler()
    cc = Controller()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        cc.run()
        rate.sleep()