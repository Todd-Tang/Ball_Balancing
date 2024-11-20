import kinematics
import math
import threading
import camera
import numpy as np
import time
import axis_control
import closed_loop_control

max_acceleration = 1000 # deg/s^2

plate_centre = camera.plate_centre
# Convert from cameraspace to robotspace
plate_centre[1] = camera.resolution[1] - plate_centre[1]

tilt_k = 0.02
max_tilt = 2.5
alpha = 0.1



[kp,ki,kd] = [1, 0.00001, -1000]
[p_sat, d_sat] = [100000, 10000000]
last_output_x = 0
last_output_y = 0
last_error_x = 0
integral_x = 0
last_error_y = 0
integral_y = 0
last_time = None



is_control_running = True


def _saturation(value, min, max):
    if value > max:
        return max
    elif value < min:
        return min
    else:
        return value

def control_robot():
    global last_time, current_time, integral_x, integral_y, last_error_x, last_error_y, last_output_x, last_output_y
    
    print("thread started")
    
    robot = kinematics.BBrobot()
    pid_x = closed_loop_control.Closed_Loop_Control(kp, ki, kd, p_sat, 0, d_sat)
    pid_y = closed_loop_control.Closed_Loop_Control(kp, ki, kd, p_sat, 0, d_sat)

    num_runs = 0

    while is_control_running:
        if camera.ball_location == None:
            continue

        num_runs += 1

        

        ball_pos = camera.ball_location
        #ball_pos = np.subtract(ball_pos, plate_centre)
        
        output_x = pid_x.update(plate_centre[0], ball_pos[0])
        output_y = pid_y.update(plate_centre[1], ball_pos[1])


        ###

        current_time = time.perf_counter()
        if last_time is None:
            last_time = current_time
        
        if current_time - last_time == 0:
            continue
        
        
        
        """""
        # 誤差を計算
        error_x = plate_centre[0] - ball_pos[0]
        error_y = plate_centre[1] - ball_pos[1]
        # 積分値を計算
        integral_x += error_x * (current_time - last_time)
        integral_y += error_y * (current_time - last_time)

        # 微分値を計算
        derivative_x = (error_x - last_error_x) / (current_time - last_time)
        derivative_y = (error_y - last_error_y) / (current_time - last_time)
        # PID出力を計算
        integral_x = 0
        integral_y = 0
        output_x = kp * error_x + ki * integral_x + kd * derivative_x
        output_y = kp * error_y + ki * integral_y + kd * derivative_y
        # ローパスフィルタを適用
        output_x = alpha * output_x + (1 - alpha) * last_output_x
        output_y = alpha * output_y + (1 - alpha) * last_output_y

        print("x: ", output_x, "y: ", output_y)
        # thetaとphiを計算
        theta = math.degrees(math.atan2(output_y, output_x))
        if theta < 0:
            theta += 360
        phi = tilt_k * math.sqrt(output_x**2 + output_y**2)
        phi = _saturation(phi, -10, 10)
        print("integral_x ", integral_x, "integral_y ", integral_y)

        last_error_x = error_x
        last_error_y = error_y
        last_output_x = output_x
        last_output_y = output_y
        last_time = current_time

        #return theta, phi
        print(theta, phi)
        """


        #
        
        theta = math.degrees(math.atan2(output_y, output_x))
        phi = -tilt_k * math.sqrt(output_x**2+output_y**2)  
        phi = _saturation(phi, -max_tilt, max_tilt)
        


        
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r*math.cos(math.radians(theta))
        y = r*math.sin(math.radians(theta))
        n = [x, y, z]

        thetas = np.subtract([90,90,90], robot.kinema_inv(n, 0.075))

        if (num_runs % 200 == 0):
            print(theta, phi)
            print(thetas)
        
        
        axis_control.move_axes(thetas[0], thetas[1], thetas[2], 1)

        time.sleep(0.05)




    


if __name__ == "__main__":

    axis_control.init_axis_control("/dev/ttyACM0", max_acceleration)
    camera_thread = threading.Thread(target=camera.detect_ball)
    camera_thread.start()
    time.sleep(0.25)
    axis_control.move_axes(90, 90, 90, 1)
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