import serial
import time

serial_port = None
m1_angle = None
m2_angle = None
m3_angle = None


def _send_command(command):
    global serial_port
    # Convert to binary first
    command = command.encode(encoding="utf-8")

    #Write out command
    serial_port.write(command + b"\n")
    serial_port.flush()
    
    #Get response
    response = serial_port.readline()
    
    print(time.time(), "> ", command)
    print(time.time(), response)

    if (b"ok" in response):
        return True
    else:
        return False

def _wait_for_move_finished():
    return _send_command("G4 P0") # Wait for move to finish


def move_axes(new_m1_angle, new_m2_angle, new_m3_angle, move_time):
    """Move all axes of a machine to the specified angles (in degrees) in the specified time (in seconds)"""

    global m1_angle, m2_angle, m3_angle

    # Convert move_time to minutes:
    move_time = move_time / 60

    # Set axis speeds
    _send_command(f"$110={abs(new_m1_angle - m1_angle) / move_time}")
    _send_command(f"$111={abs(new_m2_angle - m2_angle) / move_time}")
    _send_command(f"$112={abs(new_m3_angle - m3_angle) / move_time}")

    (m1_angle, m2_angle, m3_angle) = (new_m1_angle, new_m2_angle, new_m3_angle)

    return_value =  _send_command(f"G1 X{new_m1_angle} Y{new_m2_angle} Z{new_m3_angle} F1000") # Make the move
    _wait_for_move_finished() # Wait for the move to finish
    return return_value
    




def home_axes():
    global m1_angle, m2_angle, m3_angle

    # TODO: Add code to handle errors
    _send_command("$1 = 0")             # Turn down machine hold
    _send_command("G91")                # Set relative mode
    # For the next second, toggle between hold and no hold.
    # This is to make sure the motors fall gently
    """ Code originally made to make a "soft landing
    starting_time = time.time_ns()
    while (time.time_ns() - starting_time) < 1000000000:
        _send_command("G0 X1 Y1 Z1 F1")     # Make a move that doesn't do anything so motors fall
        time.sleep(10e-3)                          # Wait 10ms for motors to drop
    time.sleep(0.25)
    """
    _send_command("G0 X1 Y1 Z1 F1")
    time.sleep(1)
    
    # Wait for motors to fall
    _send_command("G92 X132 Y132 Z132") # Set new coords
    _send_command("G90")                # Make absolute
    _wait_for_move_finished()           # Wait for move to finish
    _send_command("$1 = 255")           # Turn on machine hold
    _send_command("G0 X131 Y131 Z131 F1")  # Back up one degree to turn on hold again
    _wait_for_move_finished()           # Wait for move to finish
    (m1_angle, m2_angle, m3_angle) = (131, 131, 131)


def init_axis_control(port_addr):
    global serial_port
    serial_port = serial.Serial(port_addr, 115200, timeout=1)
    time.sleep(1)
    print(serial_port.readall())

    # Set up constants
    # Steps per degree
    _wait_for_move_finished()
    _send_command("$100=0.555555555")
    _send_command("$101=0.555555555")
    _send_command("$102=0.555555555")
    # Invert axes
    _send_command("$3=255")
    # Turn on hold for debugging
    _send_command("$1 = 255") 
    # Home axes
    home_axes()

if __name__ == "__main__":
    init_axis_control("/dev/ttyACM0")
    move_axes(45, 80, 50, 15)
    home_axes()
    move_axes(45, 45, 45, 15)