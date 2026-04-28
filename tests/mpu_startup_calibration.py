import math
import time
from machine import I2C, Pin


_calibration = None


def has_calibration():
    return _calibration is not None


def get_calibration():
    return _calibration


def calibrate_at_startup(samples=100, i2c_freq=40000):
    global _calibration

    print("[STARTUP CAL] Initializing MPU6050...")

    i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=i2c_freq)

    from mpu6050 import MPU6050
    mpu = MPU6050(i2c)

    print("[STARTUP CAL] Keep vehicle still for MPU calibration")

    pitch_sum = 0
    yaw_bias_sum = 0
    good_samples = 0

    for _ in range(samples):
        try:
            accel = mpu.get_accel()
            gyro = mpu.get_gyro()
        except OSError:
            time.sleep(0.02)
            continue

        pitch_sum += math.degrees(math.atan2(accel["y"], accel["z"]))
        yaw_bias_sum += gyro["z"]
        good_samples += 1
        time.sleep(0.02)

    if good_samples == 0:
        print("[STARTUP CAL] Failed: no valid MPU samples")
        _calibration = None
        return None

    _calibration = {
        "pitch_offset": pitch_sum / good_samples,
        "yaw_bias": yaw_bias_sum / good_samples,
        "samples": good_samples,
        "i2c_freq": i2c_freq,
    }

    print("[STARTUP CAL] Pitch offset:", _calibration["pitch_offset"])
    print("[STARTUP CAL] Yaw bias:", _calibration["yaw_bias"])
    print("[STARTUP CAL] Calibration saved")

    return _calibration
