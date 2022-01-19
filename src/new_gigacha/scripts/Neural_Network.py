from math import atan2, radians, cos, sin, pi, degrees
import numpy as np

class Neural_Network:
    def __init__(self, state):
        self.state = state
        self.front_x = self.state.x
        self.front_y = self.state.y
        self.dis_error = 0.0
        self.ang_error = 0.0
        self.yaw = []
        self.min_index = self.state.index
        
    def distance_error(self):
        map_x = self.global_path.x[self.min_index]
        map_y = self.global_path.y[self.min_index]
        dx = map_x - self.front_x
        dy = map_y - self.front_y

        prep_vec = [cos(radians(self.state.heading)+pi/2), sin(radians(self.sate.heading)+pi/2)]
        self.dis_error = np.dot([dx, dy], prep_vec)
        

    def pid(self):
        pass

    def angle_error(self):
        map_yaw = self.yaw[self.min_index]
        final_yaw = -(map_yaw-radians(self.state.heading))
        yaw_term = self.normalize(final_yaw)
        self.ang_error = degrees(yaw_term)

    def make_yaw(self):
        for i in range(len(self.global_payh.x)-1):
            self.yaw.append(atan2(self.global_path.y[i+1]-self.global_path.y[i], self.global_path.x[i+1]-self.global_path.x[i]))

    def run(self):
        if len(self.yaw) == 0:
            self.make_yaw()

        self.distance_error()
        self.angle_error()
        