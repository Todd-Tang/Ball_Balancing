# Stepper Control Constants

MAX_ACCELERATION = 1000 # deg/s^2

# Plate motion constants
## Conversion factor from screenspace (pixel) to real-world (degrees of tilt) units
TILT_K = 0.02
## Maximum amount a plate can tilt
MAX_TILT = 2.5

# Closed Loop Configuration Constants
## Hysteresis factors for closed loop control: output ratio calculated as new/(old + new) 
ALPHA_P = 0.9