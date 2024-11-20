"""
closed_loop_control.py
Used to control the robot in a closed loop system
Implements a PD loop to control a single variable

Use 2 instances of this class to control both x and y

"""


import time
from time import sleep
import math

class Closed_Loop_Control:

    def __init__(kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # TODO: Implement Ki if necessary, but otherwise it will be left blank.

        self.error = 0
        self.last_error = 0
        self.last_time = time.time()
        self.current_time = time.time()


    def update(self, setpoint, current_value):
        self.current_time = time.time()
        if self.last_time is None:
            self.last_time = self.current_time
            
        if self.current_time - self.last_time == 0:
            return 0

        # Calculate error
        self.error = setpoint - current_value

        # Calculate derivative
        derivative = (self.error - self.last_error) / (self.current_time - self.last_time)

        # Calculate output
        output = self.kp * self.error + self.kd * derivative

        # Update variables
        self.last_error = self.error
        self.last_time = self.current_time

        return output




