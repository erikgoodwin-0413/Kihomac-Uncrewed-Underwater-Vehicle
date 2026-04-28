# tests/pid.py

import time


class PID:
    def __init__(self, kp, ki, kd, limits=(-400, 400)):
        # gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # output limits
        self.min_out, self.max_out = limits

        # state
        self.integral = 0
        self.prev_error = 0
        self.prev_time = None
        self.d_prev = 0

        # tuning helpers
        self.integral_limit = 100      # anti-windup clamp
        self.d_filter_alpha = 0.7      # derivative smoothing

    # -----------------------
    # RESET (IMPORTANT)
    # -----------------------
    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.prev_time = None
        self.d_prev = 0

    # -----------------------
    # UPDATE
    # -----------------------
    def update(self, error):
        now = time.ticks_ms()

        # first run
        if self.prev_time is None:
            self.prev_time = now
            self.prev_error = error
            return 0

        dt = time.ticks_diff(now, self.prev_time) / 1000

        if dt <= 0:
            return 0

        # -----------------------
        # INTEGRAL (with anti-windup)
        # -----------------------
        self.integral += error * dt
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral)
        )

        # -----------------------
        # DERIVATIVE (with filtering)
        # -----------------------
        raw_derivative = (error - self.prev_error) / dt

        derivative = (
            self.d_filter_alpha * self.d_prev +
            (1 - self.d_filter_alpha) * raw_derivative
        )

        self.d_prev = derivative

        # -----------------------
        # PID OUTPUT
        # -----------------------
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        # clamp output
        output = max(self.min_out, min(self.max_out, output))

        # update state
        self.prev_error = error
        self.prev_time = now

        return output

    # -----------------------
    # OPTIONAL: SET LIMITS
    # -----------------------
    def set_limits(self, min_out, max_out):
        self.min_out = min_out
        self.max_out = max_out

    # -----------------------
    # OPTIONAL: SET GAINS
    # -----------------------
    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd