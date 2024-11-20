import axis_control
from time import sleep

axis_control.init_axis_control("/dev/ttyACM0", 10, logging=True)

# Try to test acceleration figures
acceleration_max = 1995
acceleration_min = 0

do_log = False

# Use a binary search to find the maximum acceleration
while acceleration_max - acceleration_min > 1:
    
    acceleration = (acceleration_max + acceleration_min) / 2
    print(f"Trying acceleration: {acceleration}deg/s^2")

    axis_control.home_axes(logging=do_log)
    axis_control.set_acceleration(acceleration, logging=do_log)
    
    for i in range(10):
        axis_control.move_axes(120, 120, 120, 1, logging=do_log)
        sleep(1)
        axis_control.move_axes(35, 35, 35, 1, logging=do_log)
        sleep(1)

    axis_control.move_axes(90,90,90, 1, logging=do_log)

    user_input = input("Is the acceleration too high? (y/n): ")
    if "y" in user_input:
        acceleration_max = acceleration
    elif "r" in user_input:
        continue
    else:
        acceleration_min = acceleration

   


