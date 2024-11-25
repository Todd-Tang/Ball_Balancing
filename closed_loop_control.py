"""
closed_loop_control.py
Used to control the robot in a closed loop system
Implements a PD loop to control a single variable

Use 2 instances of this class to control both x and y

"""


import time
from time import sleep
import math


def _saturate(value, min_value, max_value):
    if value > max_value:
        return max_value
    elif value < min_value:
        return min_value
    else:
        return value

class Closed_Loop_Control:

    def __init__(self, kp, ki, kd, sat_p, sat_i, sat_d, alpha):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.sat_p = sat_p
        self.sat_i = sat_i
        self.sat_d = sat_d

        self.alpha = alpha

        # TODO: Implement Ki if necessary, but otherwise it will be left blank.

        self.error = 0
        self.last_error = 0
        self.integral = 0 
        self.last_time = time.time()
        self.current_time = time.time()

        self.epsilon = 0
        self.last_output = 0

        self.counter = 0
        self.last_5th_error = 0
        self.last_5th_time = 0


    def update(self, setpoint, current_value):
        self.current_time = time.time()
        if self.last_time is None:
            self.last_time = self.current_time
            
        if self.current_time - self.last_time == 0:
            return 0

        # Calculate error
        self.error = setpoint - current_value

        if abs(self.error) < abs(self.epsilon):
            return 0
        
        # Calculate integral
        self.integral += self.error * (self.current_time - self.last_time)
        # Saturate integral
        self.integral = _saturate(self.integral, -self.sat_i / self.ki, self.sat_i/self.ki)

        if self.counter == 5:
            derivative = (self.error - self.last_5th_error) / (self.current_time - self.last_5th_time)
            self.last_5th_error = self.error
            self.last_5th_time = self.current_time
            self.counter = 0

        self.counter += 1
        
        
        # derivative = (self.error - self.last_error) / (self.current_time - self.last_time)

        # Calculate derivative

        p_component = _saturate(self.kp * self.error, -self.sat_p, self.sat_p)

        i_component = self.ki * self.integral
    
        d_component = _saturate(self.kd * derivative, -self.sat_d, self.sat_d)


        # Calculate output
        print(f"p_component: {p_component}, i_component: {i_component}, d_component: {d_component}")

        output = (p_component + i_component + d_component) * (1-self.alpha) + (self.alpha) * self.last_output
        self.last_output = output


        # Update variables
        self.last_error = self.error
        self.last_time = self.current_time

        return output




