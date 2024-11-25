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
        self.last_error = []
        self.v = []
        self.integral = 0 
        self.derivative = 0
        self.last_time = []
        self.current_time = time.time()

        self.epsilon = 0
        self.last_output = 0

        self.counter = 0


    def update(self, setpoint, current_value):
        self.current_time = time.time()
        if len(self.last_time) == 0:
            self.error = setpoint - current_value
            self.last_time.append(self.current_time)
            self.last_error.append(self.error)
            print(self.last_error, self.last_time)  
            self.counter += 1
            return 0
        

        #if self.current_time - self.last_time == 0:
        #if self.counter == 0:
        #    return 0

        # Calculate error
        self.error = setpoint - current_value

        # Check if the data has changed and if it hasn't, skip this loop.
        # Index is -1 since no new data has yet been added to the list. 
        # if self.error == self.last_error[-1]:
        #     return 0

        self.last_error.append(self.error)
        self.last_time.append(self.current_time)


        if abs(self.error) < abs(self.epsilon):
            return 0
        
        # Calculate integral
        self.integral += self.error * (self.current_time - self.last_time[-2])
        # Saturate integral
        self.integral = _saturate(self.integral, -self.sat_i / self.ki, self.sat_i/self.ki)

        """print(self.last_error, self.last_time)"""  

        if self.counter <= 4:
            self.derivative = (self.error - self.last_error[-2]) / (self.current_time - self.last_time[-2])
            self.v.append(self.derivative)
            self.counter += 1
        else:
            for i in range(4):
                self.v[i] = (self.last_error[i+1]-self.last_error[i])/(self.last_time[i+1]-self.last_time[i])
            self.derivative = self.v[0]*0.1 + self.v[1]* 0.1 + self.v[2]*0.2 + self.v[3]*0.6
        
            """print(f"Velocity: [{self.v[0]},{self.v[1]},{self.v[2]},{self.v[3]}]")"""


        # derivative = (self.error - self.last_error) / (self.current_time - self.last_time)

        # Calculate derivative

        p_component = _saturate(self.kp * self.error, -self.sat_p, self.sat_p)

        i_component = self.ki * self.integral
    
        d_component = _saturate(self.kd * self.derivative, -self.sat_d, self.sat_d)


        # Calculate output
        """print(f"p_component: {p_component}, i_component: {i_component}, d_component: {d_component}")"""

        output = (p_component + i_component + d_component) * (1-self.alpha) + (self.alpha) * self.last_output
        self.last_output = output


        # Update variables


        #self.last_error = self.error
        #self.last_time = self.current_time

        
        if len(self.last_error) >= 5:
            self.last_error.pop(0)
            self.last_time.pop(0)

        return output




