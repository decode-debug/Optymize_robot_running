import math

class PIDController:
    def __init__(self, kp, ki, kd, dt,
                 integral_limit=None,
                 use_angle=False,
                 d_alpha=0.0,
                 derivative_on_measurement=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.previous_error = 0.0

        # Options / improvements
        self.integral_limit = integral_limit      # e.g., 100.0 (units of the integral)
        self.use_angle = use_angle                # True -> angle error (-pi..pi]
        self.d_alpha = float(d_alpha)             # 0..1, low-pass filter for D term
        self.derivative_on_measurement = derivative_on_measurement

        # Internal states for options
        self._previous_measured = None            # for D on measurement
        self._previous_derivative = 0.0           # for D filtering

    @staticmethod
    def _wrap_angle_rad(x):
        """Wrap angle to (-pi, pi]."""
        return math.atan2(math.sin(x), math.cos(x))

    @staticmethod
    def _clamp(x, lo, hi):
        """Clamp x to [lo, hi]."""
        return max(lo, min(hi, x))

    def reset(self, integral=0.0, previous_error=0.0):
        """Reset integral/derivative states."""
        self.integral = float(integral)
        self.previous_error = float(previous_error)
        self._previous_measured = None
        self._previous_derivative = 0.0

    def compute(self, setpoint, measured_value):
        if measured_value == 0:
            return 0
        
        # 1) Error (optionally wrapped for angular variables)
        if self.use_angle:
            error = self._wrap_angle_rad(setpoint - measured_value)
        else:
            error = setpoint - measured_value

        # 2) Integral with anti-windup (clamp)
        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = self._clamp(self.integral, -self.integral_limit, self.integral_limit)

        # 3) Derivative (backward difference) — optionally on measurement, then low-pass filtered
        if self.dt > 0:
            if self.derivative_on_measurement:
                if self._previous_measured is None:
                    raw_derivative = 0.0
                else:
                    dm = measured_value - self._previous_measured
                    if self.use_angle:
                        dm = self._wrap_angle_rad(dm)
                    raw_derivative = - dm / self.dt  # D on measurement => negative sign
                self._previous_measured = measured_value
            else:
                de = error - self.previous_error
                raw_derivative = de / self.dt
        else:
            raw_derivative = 0.0

        # Low-pass filter for D: d = (1 - a) * raw + a * prev
        derivative = (1.0 - self.d_alpha) * raw_derivative + self.d_alpha * self._previous_derivative
        self._previous_derivative = derivative

        # 4) PID output
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # 5) Update state
        self.previous_error = error

        return output

#!/usr/bin/env python3
from time import sleep
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent
from ev3dev2.motor import MoveDifferential
from ev3dev2.wheel import EV3Tire

# Initialize the color sensors and robot
left_color_sensor  = ColorSensor(INPUT_1)
right_color_sensor = ColorSensor(INPUT_2)
left_motor = LargeMotor(OUTPUT_A)  # Connect to port A (left motor)
right_motor = LargeMotor(OUTPUT_B) # Connect to port B (right motor)
robot = MoveDifferential(left_motor, right_motor, EV3Tire, 114)

# Base speeds
speed = SpeedPercent(25)
speed_on_turn = SpeedPercent(20)

# PID setup: tune these
pid = PIDController(kp=1.2, ki=0.0, kd=0.2, dt=0.05,
                    integral_limit=100, d_alpha=0.2)

# Line calibration values
BLACK = 20
WHITE = 80
TARGET = (BLACK + WHITE) / 2

try:
    print("PID line following with degree steering...")
    while True:
        L = left_color_sensor.reflected_light_intensity
        R = right_color_sensor.reflected_light_intensity

        # Error = difference between left and right intensity
        error = R - L  # positive -> line on right, turn right

        # PID output: in degrees to turn
        turn_degrees = pid.compute(0, error)

        # Clamp to reasonable range
        turn_degrees = max(-20, min(20, turn_degrees))

        # Decide direction
        if turn_degrees > 1:
            robot.turn_right(speed_on_turn, turn_degrees, brake=False, block=True)
        elif turn_degrees < -1:
            robot.turn_left(speed_on_turn, abs(turn_degrees), brake=False, block=True)
        else:
            robot.on_for_seconds(speed, 0.05, brake=False, block=True)

        sleep(0.05)

except KeyboardInterrupt:
    robot.off()
    print("Stopped.")
