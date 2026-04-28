import time
import math
from machine import I2C, Pin
from tests.base_test import SubTestBase
from tests.pid import PID


# -----------------------
# FILTER
# -----------------------
class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = 0

    def update(self, x):
        self.value = x if self.value is None else (self.alpha * x + (1 - self.alpha) * self.value)
        return self.value


class AttitudePIDTest(SubTestBase):
    name = "Pitch + Yaw (Gyro Estimated)"

    def __init__(self):
        super().__init__()

        self.i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)

        from mpu6050 import MPU6050
        self.mpu = MPU6050(self.i2c)

        self.led = Pin(25, Pin.OUT)

        # -----------------------
        # FILTERS
        # -----------------------
        self.pitch_f = LowPassFilter(0.08)
        self.yaw_f   = LowPassFilter(0.05)

        # -----------------------
        # PID
        # -----------------------
        self.pitch_pid = PID(1.2, 0.0, 0.15)
        self.yaw_pid   = PID(1.2, 0.0, 0.15)

        # -----------------------
        # CALIBRATION
        # -----------------------
        self.pitch_offset = 0
        self.yaw_bias = 0

        # yaw integration state
        self.yaw = 0
        self.last_t = None

        # -----------------------
        # STATE
        # -----------------------
        self.running = False

        # -----------------------
        # FIN SETTINGS
        # -----------------------
        self.neutral = 307
        self.fin_gain = 15

        self.min_pwm = 225
        self.max_pwm = 389

        # smoothing
        self.alpha_out = 0.3
        self.last_up = self.neutral
        self.last_down = self.neutral
        self.last_left = self.neutral
        self.last_right = self.neutral

    # -----------------------
    # START
    # -----------------------
    def start(self):
        print("Starting Pitch + Yaw Controller")

        self.safe_stop_all()
        self.fins_neutral()

        self.calibrate()

        self.last_t = time.ticks_ms()
        self.running = True

        return True

    # -----------------------
    # CALIBRATION
    # -----------------------
    def calibrate(self, samples=100):
        print("Calibrating... keep still")

        ps = 0
        yaw_bias_sum = 0

        for _ in range(samples):
            a = self.mpu.get_accel()
            g = self.mpu.get_gyro()

            ps += self._pitch(a)
            yaw_bias_sum += g['z']   # yaw rate bias

            time.sleep(0.02)

        self.pitch_offset = ps / samples
        self.yaw_bias = yaw_bias_sum / samples

        print("Pitch offset:", self.pitch_offset)
        print("Yaw bias:", self.yaw_bias)

    # -----------------------
    # UPDATE LOOP
    # -----------------------
    def update(self):
        if not self.running:
            return

        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_t) / 1000
        self.last_t = now

        if dt <= 0:
            return

        accel = self.mpu.get_accel()
        gyro  = self.mpu.get_gyro()

        # -----------------------
        # PITCH (STABLE)
        # -----------------------
        pitch = self._pitch(accel) - self.pitch_offset
        pitch = self.pitch_f.update(pitch)

        # -----------------------
        # YAW (GYRO INTEGRATION)
        # -----------------------
        yaw_rate = gyro['z'] - self.yaw_bias
        self.yaw += yaw_rate * dt
        yaw = self.yaw_f.update(self.yaw)

        # -----------------------
        # PID
        # -----------------------
        pitch_cmd = self.pitch_pid.update(-pitch)
        yaw_cmd   = self.yaw_pid.update(-yaw)

        self.set_fins(pitch_cmd, yaw_cmd)

        if time.ticks_ms() % 200 < 20:
            print("Pitch:", pitch, "Yaw:", yaw)

        time.sleep(0.02)

    # -----------------------
    # FIN CONTROL
    # -----------------------
    def set_fins(self, pitch_cmd, yaw_cmd):

        pitch_cmd *= self.fin_gain * 0.4
        yaw_cmd   *= self.fin_gain * 0.4

        # -----------------------
        # PITCH → HORIZONTAL FINS
        # -----------------------
        left  = self.neutral - pitch_cmd
        right = self.neutral + pitch_cmd

        # -----------------------
        # YAW → VERTICAL FINS
        # -----------------------
        up    = self.neutral - yaw_cmd
        down  = self.neutral + yaw_cmd

        def clamp(x):
            return max(self.min_pwm, min(self.max_pwm, int(x)))

        self.last_left  += self.alpha_out * (left - self.last_left)
        self.last_right += self.alpha_out * (right - self.last_right)
        self.last_up    += self.alpha_out * (up - self.last_up)
        self.last_down  += self.alpha_out * (down - self.last_down)

        self.set_pwm(self.LEFT_FIN,  clamp(self.last_left))
        self.set_pwm(self.RIGHT_FIN, clamp(self.last_right))
        self.set_pwm(self.UP_FIN,    clamp(self.last_up))
        self.set_pwm(self.DOWN_FIN,  clamp(self.last_down))

    # -----------------------
    # ANGLES
    # -----------------------
    def _pitch(self, a):
        return math.degrees(math.atan2(a['y'], a['z']))

    # -----------------------
    # STOP
    # -----------------------
    def stop(self):
        self.running = False
        self.safe_stop_all()
        self.fins_neutral()
        print("Stopped")


# -----------------------
# RUN
# -----------------------
if __name__ == "__main__":

    t = AttitudePIDTest()

    if t.start():
        try:
            while True:
                t.update()
        except KeyboardInterrupt:
            pass
        finally:
            t.stop()
