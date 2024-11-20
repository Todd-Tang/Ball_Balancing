import serial
import time
import math
import threading

microstep_setting = 16
steps_per_degree = 0.555555555 * microstep_setting

serial_port = None

target_m1_angle = None
target_m2_angle = None
target_m3_angle = None

m1_angle = None
m2_angle = None
m3_angle = None

min_move = 0.1
max_move = 10

is_running = True
is_homing = False

def _saturate(val, min_val, max_val):
    return min(max(val, min_val), max_val)

def _send_command(command, logging=False):
    global serial_port
    # Convert to binary first
    command = command.encode(encoding="utf-8")

    #Write out command
    serial_port.write(command + b"\n")
    serial_port.flush()
    
    #Get response
    response = serial_port.readline()
    
    if logging:
        print(time.time(), "> ", command)
        print(time.time(), response)

    if (b"ok" in response):
        return True
    else:
        return False

def _wait_for_move_finished(logging=False):
    return _send_command("G4 P0", logging) # Wait for move to finish

def move_axes(new_m1_angle, new_m2_angle, new_m3_angle, move_time, logging=False):
    """Submit a new location for the machine to move to"""
    global target_m1_angle, target_m2_angle, target_m3_angle

    (target_m1_angle, target_m2_angle, target_m3_angle) =(_saturate(new_m1_angle, 30, 132), _saturate(new_m2_angle, 30, 132), _saturate(new_m3_angle, 30, 132))

def _move_axes(logging = False):
    """Move the machine to the target location"""
 
    global m1_angle, m2_angle, m3_angle

    if target_m1_angle == None or target_m2_angle == None or target_m3_angle == None or m1_angle == None or m2_angle == None or m3_angle == None:
        return True

    delta = [target_m1_angle - m1_angle, target_m2_angle - m2_angle, target_m3_angle - m3_angle]

    move_size = math.sqrt(delta[0]**2 + delta[1]**2 + delta[2]**2)

    # Check to see if the move is big enough to be worth it
    if move_size > min_move:
    
    # If the move is too big, scale it down
        if move_size > max_move:
            delta = [x * max_move / move_size for x in delta]
            move_size = max_move

        
        [m1_angle, m2_angle, m3_angle] = [m1_angle + delta[0], m2_angle + delta[1], m3_angle + delta[2]]

        return_value = _send_command(f"G1 Z{m1_angle} Y{m2_angle} X{m3_angle} F100000", logging) # Make the move
        _wait_for_move_finished() # Wait for the move to finish
        return return_value
    
    return True

def set_acceleration(acceleration, logging=False):
    ret_val = _send_command(f"$120 = {acceleration}", logging)
    ret_val &= _send_command(f"$121 = {acceleration}", logging)
    ret_val &= _send_command(f"$122 = {acceleration}", logging)
    return ret_val


def home_axes(logging=False):
    global m1_angle, m2_angle, m3_angle, is_homing

    # TODO: Add code to handle errors
    _send_command("$1 = 0", logging)             # Turn down machine hold
    _send_command("G91", logging)                # Set relative mode
    # For the next second, toggle between hold and no hold.
    # This is to make sure the motors fall gently

    is_homing = True
    
    # # Code originally made to make a "soft landing"
    # starting_time = time.time_ns()
    # while (time.time_ns() - starting_time) < 1000000000:
    #     _send_command("G0 X1 Y1 Z1 F1")     # Make a move that doesn't do anything so motors fall
    #     time.sleep(10e-3)                          # Wait 10ms for motors to drop
    # time.sleep(0.25)

    
    # Move to the home position
    _send_command("G0 X1 Y1 Z1 F1", logging)
    time.sleep(1)
    
    
    # Wait for motors to fall
    _send_command("G92 X132 Y132 Z132", logging) # Set new coords
    _send_command("G90", logging)                # Make absolute
    _wait_for_move_finished(logging)           # Wait for move to finish
    _send_command("$1 = 255", logging)           # Turn on machine hold
    _send_command("G0 X131 Y131 Z131 F1", logging)  # Back up one degree to turn on hold again
    _wait_for_move_finished(logging)           # Wait for move to finish
    (m1_angle, m2_angle, m3_angle) = (131, 131, 131)

    is_homing = False

def axis_control_loop():
    while is_running:
        if is_homing:
            time.sleep(0)
            continue
        _move_axes()
        time.sleep(0)

def init_axis_control(port_addr, max_acceleration, logging=False):
    global serial_port
    serial_port = serial.Serial(port_addr, 115200, timeout=1)
    time.sleep(1)
    print(serial_port.readall())

    # Set up constants
    # Steps per degree
    _wait_for_move_finished()
    _send_command(f"$100={steps_per_degree}", logging)
    _send_command(f"$101={steps_per_degree}", logging)
    _send_command(f"$102={steps_per_degree}", logging)

    # Set up acceleration
    set_acceleration(max_acceleration, logging)

    # Invert axes
    _send_command("$3=255", logging)
    # Turn on hold for debugging
    _send_command("$1 = 255",logging) 
    # Home axes
    home_axes()

    # Motor control loop
    motor_control_thread = threading.Thread(target=axis_control_loop)
    motor_control_thread.start()

# Stops the thread
def stop():
    global is_running
    is_running = False


if __name__ == "__main__":
    init_axis_control("/dev/ttyACM0")
    move_axes(45, 45, 45, 15)
    home_axes()
    move_axes(90, 90, 90, 15)
  