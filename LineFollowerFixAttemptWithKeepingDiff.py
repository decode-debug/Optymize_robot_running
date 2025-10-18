#!/usr/bin/env python3
from time import sleep
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, MoveTank, SpeedPercent

# Initialize color sensors
left_color_sensor = ColorSensor(INPUT_1)   # Left color sensor (Port 1)
right_color_sensor = ColorSensor(INPUT_2)  # Right color sensor (Port 2)

# Initialize motors
left_motor = LargeMotor(OUTPUT_A)   # Left motor (Port A)
right_motor = LargeMotor(OUTPUT_B)  # Right motor (Port B)
robot = MoveTank(OUTPUT_A, OUTPUT_B)

# Define base speeds
speed = 25  # forward base speed (%)
speed_on_turn = 25  # turning base speed (%)

# Define black and white calibration values (adjust for your mat)
BLACK = 20
WHITE = 80
TARGET = (BLACK + WHITE) / 2  # midpoint between black and white

# Controller settings
Kp = 0.9      # proportional gain (tune for smooth following)
MAX_CORR = 40  # clamp correction

# Helper function
def clamp(x, low, high):
    return max(low, min(high, x))

# Start the line following loop
try:
    print("Starting line following... (press Ctrl+C to stop)")
    while True:
        # Read reflected light intensity from both sensors
        left_value = left_color_sensor.reflected_light_intensity
        right_value = right_color_sensor.reflected_light_intensity

        # Compute proportional error based on difference from target
        error_left = left_value - TARGET
        error_right = right_value - TARGET
        error = error_right - error_left

        # Compute correction (proportional term)
        correction = clamp(Kp * error, -MAX_CORR, MAX_CORR)

        # Adjust motor speeds
        left_speed = clamp(speed - correction, -100, 100)
        right_speed = clamp(speed + correction, -100, 100)

        # Apply speeds to motors
        robot.on(SpeedPercent(left_speed), SpeedPercent(right_speed))

        # Short delay to keep loop stable
        sleep(0.01)

except KeyboardInterrupt:
    print("Line following stopped by user.")
    robot.off()
