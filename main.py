import time
import math

import numpy as np
import cv2

import kinematics
import threading
import camera

import axis_control
import closed_loop_control


max_acceleration = 700 # deg/s^2

plate_centre = camera.plate_centre.copy()
# Convert from cameraspace to robotspace
plate_centre[1] = camera.resolution[1] - plate_centre[1]

tilt_k = 0.02
max_tilt = 8
alpha = 0.1



#[kp,ki,kd] = [1.5, 0.5, 1.5]
[kp,ki,kd] = [1.5, 0.5, 1.5]
[p_sat, i_sat, d_sat] = [100000, 100, 10000000]
last_output_x = 0
last_output_y = 0
last_error_x = 0
integral_x = 0
last_error_y = 0
integral_y = 0
last_time = None



is_control_running = True


# Trajectory generation
is_running_trajectory = True
trajectory = [[50,50], [-50, 50], [-50, -50], [50, -50]]
trajectory_wait_time = 5 #seconds
trajectory_index = 0
last_transition_time = time.time()
target_location = [522,350]

def _saturation(value, min, max):
    if value > max:
        return max
    elif value < min:
        return min
    else:
        return value

def control_robot():
    global last_time, current_time, integral_x, integral_y, last_error_x, last_error_y, last_output_x, last_output_y, target_location
    
    print("thread started")
    
    robot = kinematics.Kinematics()
    pid_x = closed_loop_control.Closed_Loop_Control(kp, ki, kd, p_sat, i_sat, d_sat, alpha)
    pid_y = closed_loop_control.Closed_Loop_Control(kp, ki, kd, p_sat, i_sat, d_sat, alpha)

    num_runs = 0

    while is_control_running:
        if camera.ball_location == None:
            continue

        num_runs += 1

        target_location = plate_centre

        if is_running_trajectory:
            global trajectory_index, last_transition_time
            time_factor  = time.time() / 10

            target_location = np.add([math.cos(time_factor), math.sin(time_factor)], plate_centre)

            # # interpolate target_trajectory
            # interpolation_factor = (time.time() - last_transition_time) / trajectory_wait_time

            # target_location = np.add(trajectory[trajectory_index], np.multiply(np.subtract(trajectory[(trajectory_index + 1) % len(trajectory)], trajectory[trajectory_index]), interpolation_factor))

            # target_location = np.add(target_location, plate_centre)

            # Update camera centre location
            camera.set_target_location(target_location)

            print(f"target_location {target_location}")

            if time.time() - last_transition_time > trajectory_wait_time:
                trajectory_index = (trajectory_index + 1) % len(trajectory)
                last_transition_time = time.time()
                pid_x.zero_variables()
                pid_y.zero_variables()


        ball_pos = camera.ball_location
        #ball_pos = np.subtract(ball_pos, plate_centre)
        
        output_x = pid_x.update(target_location[0], ball_pos[0])
        output_y = pid_y.update(target_location[1], ball_pos[1])

        # img = np.zeros((1280,720,3), dtype="uint8")
        # # draw goal
        # cv2.circle(img, target_location.astype(np.uint8), 5, (255, 0, 0), -1)

        # # draw deadzone
        # cv2.circle(img, target_location.astype(np.uint8), 15, (255, 0, 0), 2)
        # print(target_location)

        # cv2.circle(img, (int(ball_pos[0]), int(ball_pos[1])), 5, (0, 0, 255), -1)

        # cv2.imshow("img", img) 

        ###

        current_time = time.perf_counter()
        if last_time is None:
            last_time = current_time
        
        if current_time - last_time == 0:
            continue
        
    



        #
        
        theta = math.degrees(math.atan2(output_y, output_x))
        phi = -tilt_k * math.sqrt(output_x**2+output_y**2)  
        phi = _saturation(phi, -max_tilt, max_tilt)
        


        
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r*math.cos(math.radians(theta))
        y = r*math.sin(math.radians(theta))
        n = [x, y, z]

        thetas = np.subtract([90,90,90], robot.inv_kinematics(n, 0.075))

        if (num_runs % 200 == 0):
            print(theta, phi)
            print(thetas)
        
        
        axis_control.move_axes(thetas[0], thetas[1], thetas[2], 1)

        time.sleep(0.05)




    


if __name__ == "__main__":

    axis_control.init_axis_control("/dev/ttyACM0", max_acceleration)
    
    axis_control.set_acceleration(100)
    axis_control.move_axes(90, 90, 90, 1)
    time.sleep(4)
    axis_control.set_acceleration(max_acceleration)
    camera_thread = threading.Thread(target=camera.detect_ball, args=[target_location])
    camera_thread.start()
    time.sleep(0.25)
    robot_thread = threading.Thread(target=control_robot)
    robot_thread.start()

    while is_control_running:
        user_input = input("Press q to quit: ")
        if "q" in user_input:
            camera.stop()
            camera_thread.join()

            is_control_running = False
            robot_thread.join()

            axis_control.move_axes(130, 130, 130, 1)
            time.sleep(1.5)
            
            axis_control.is_running = False
            break





# Robot = kinematics.BBrobot()

# phi = 10
# theta = 60

# z = math.cos(math.radians(phi))
# r = math.sin(math.radians(phi))
# x = r*math.cos(math.radians(theta))
# y = r*math.sin(math.radians(theta))
# n = [x, y, z]
# print(n)

# thetas = Robot.kinema_inv(n,0.075)
# print(90-thetas[0],90-thetas[1],90-thetas[2])