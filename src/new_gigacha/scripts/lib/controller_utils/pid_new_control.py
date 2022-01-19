import errno
from math import atan2, cos, radians, sin, pi, degrees
import numpy as np
import pandas as pd

class PID_C:
    def __init__(self, state, global_path, local_path):
        self.dis_error = []
        self.ang_error = []
        self.state = state
        self.global_path = global_path
        # self.kp_go = 0.07
        # self.ki_go = 0.0001
        # self.kd_go = 3.0
        self.kp_steer = 0.07
        self.ki_steer = 0.0001
        self.kd_steer = 3.0
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        self.total_error = 0.2
        self.numsteps = 200
        self.step_num = 1
        self.yaw = []
        self.best_err = 0
        # self.p_go = [self.kp_go*0.1, self.ki_go*0.1, self.kd_go*0.1]
        self.p_steer = [self.kp_steer*0.1, self.ki_steer*0.1, self.kd_steer*0.1]
        self.p_index = 0
        self.steer = state.steer
        self.count = 0

    def make_yaw(self):
        for i in range(len(self.global_path.x)-1):
            self.yaw.append(atan2(self.global_path.y[i+1]-self.global_path.y[i],\
                 self.global_path.x[i+1]-self.global_path.x[i]))

    def normalize(self, angle):
        while angle > pi:
            angle -= 2.0 * pi

        while angle < -pi:
            angle += 2.0 * pi

        return angle

    def make_csv(self, cte, ane):
        f = open('PID_error.csv', 'w')

        # for i in range(len(self.dis_error)):
        #     f.write(cte + ',' + ane + ',' + self.kp_steer + ',' + self.kd_steer + ',' + self.ki_steer + '\n')

    def distance_error(self, min_index):
        map_x = self.global_path.x[min_index]
        map_y = self.global_path.y[min_index]

        front_x = self.state.x
        front_y = self.state.y

        dx = map_x - front_x
        dy = map_y - front_y

        prep_vec = [cos(radians(self.state.heading)+pi/2), sin(radians(self.state.heading)+pi/2)]
        now_error = np.dot([dx, dy], prep_vec)
        self.dis_error.append(now_error)
        return now_error

    def angle_error(self, min_index):
        map_yaw = self.yaw[min_index]
        final_yaw = -(map_yaw-radians(self.state.heading))
        yaw_term = self.normalize(final_yaw)
        now_error = degrees(yaw_term)
        self.ang_error.append(now_error)
        return now_error

    def update_error(self, cte, p, what):
        if self.count == 0:
            self.p_error = cte
            self.count += 1

        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte

        self.best_err += cte**2

        p = [0, 0, 0]
        dp = [1, 1, 1]
        
        it = 0
        while sum(dp) > self.total_error:
            for i in range(len(p)):
                p[i] += dp[i]
            
                # print(dp)
                if cte < self.best_err:
                    self.best_err = cte
                    dp[i] *= 1.1
                else:
                    p[i] -= 2 * dp[i]

                    if cte < self.best_err:
                        self.best_err = cte
                        dp[i] *= 1.1
                    else:
                        p[i] += dp[i]
                        dp[i] *= 0.9
                it += 1
        
        self.kp_steer = p[0]
        self.kd_steer = p[1]
        self.ki_steer = p[2]
        print("---------ok------")

    def run(self):
        if len(self.yaw) == 0:
            self.make_yaw()

        min_index = self.state.index
        print(min_index)

        cte = self.distance_error(min_index)
        ane = self.angle_error(min_index)

        self.update_error(cte, self.p_steer, False)
        self.make_csv(cte, ane)
        a = -self.kp_steer*self.p_error-self.kd_steer*self.d_error
        print("state_steer : ")
        print(self.state.steer)
        print('\n kp_steer : ') # pid gain
        print(self.kp_steer) 
        print('\n p_error : ') # error
        print(self.p_error)
        print('\n kd_steer : ')
        print(self.kd_steer)
        print('\n d_error : ')
        print(self.d_error)
        print('\n ki_steer : ')
        print(self.ki_steer)
        print('\n i_error : ')
        print(self.i_error)
        print('\n final_steer : ')

        print(a)
        return max(min(a, 27.0), -27.0)