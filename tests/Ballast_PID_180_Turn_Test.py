import math
import time
import ms5837
from machine import I2C, Pin
from tests.base_test import SubTestBase
from tests.mpu_startup_calibration import get_calibration, has_calibration
from tests.pid import PID


class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, x):
        self.value = x if self.value is None else (
            self.alpha * x + (1 - self.alpha) * self.value
        )
        return self.value

    def reset(self):
        self.value = None


class BallastPID180TurnTest(SubTestBase):
    name = "Ballast + PID 180 Turn"

    def __init__(self):
        super().__init__()

        # Thruster / ballast
        self.NEUTRAL_US = 1500
        self.FORWARD_US = 1800
        self.TURNDOWN_US = 1600
        self.BALLAST_CLOSED = 180
        self.BALLAST_OPEN = 0

        # Raw PCA9685 fin values used by the ballast mission
        self.FIN_NEUTRAL = 307
        self.FIN_DEFLECT_RIGHT = 360
        self.FIN_DEFLECT_LEFT = 254
        self.min_pwm = 225
        self.max_pwm = 389

        # Mission state
        self.phase = 0
        self.phase_start = 0
        self.start_ms = 0
        self.file = None
        self.current_thruster_us = self.NEUTRAL_US
        self.last_ramp_ms = 0
        self.ramp_step = 10
        self.ramp_delay_ms = 100
        self.phase6_rampdown = False

        # IMU / PID state
        self.mpu = None
        self.pitch_f = LowPassFilter(0.08)
        self.yaw_f = LowPassFilter(0.05)
        self.pitch_pid = PID(1.2, 0.0, 0.15)
        self.yaw_pid = PID(1.2, 0.0, 0.15)
        self.pitch_offset = 0
        self.yaw_bias = 0
        self.yaw = 0
        self.last_t = None
        self.yaw_target = 0
        self.locked_180 = False
        self.lock_band = 6
        self.unlock_band = 12
        self.turn_lock_start = None
        self.turn_lock_hold_s = 1.0
        self.fin_gain = 15
        self.alpha_out = 0.3
        self.last_up = self.FIN_NEUTRAL
        self.last_down = self.FIN_NEUTRAL
        self.last_left = self.FIN_NEUTRAL
        self.last_right = self.FIN_NEUTRAL
        self.last_pitch = 0
        self.last_yaw = 0
        self.last_yaw_rate = 0
        self.last_yaw_error = 0
        self.last_status_ms = 0

        # MPU orientation correction.
        # The board is now mounted rightside up, so no polarity inversion
        # is needed for pitch or yaw-rate.
        self.MPU_PITCH_SIGN = 1
        self.MPU_YAW_RATE_SIGN = 1

    def start(self):
        print("Ballast + PID 180 Turn START")
        print("[INIT] Bringing up I2C / PCA9685 / sensors")

        if self.i2c is None:
            self.i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=40000)
            self.init_pca()
            print("[INIT] PCA9685 ready")

        self.sensor = ms5837.MS5837_02BA(self.i2c)
        self.sensor.setFluidDensity(1.0108099)
        if not self.sensor.init():
            print("Sensor init failed")
            return False
        print("[INIT] Pressure sensor ready")

        from mpu6050 import MPU6050
        self.mpu = MPU6050(self.i2c)
        print("[INIT] MPU6050 ready")

        if not self._load_startup_calibration():
            print("Startup MPU calibration not available")
            return False

        self.safe_stop_all()
        self.solenoid_off()
        self.fins_neutral_ticks()
        self.set_ballast(self.BALLAST_CLOSED)

        self.file = open("ballast_pid_180_turn_log.csv", "w")
        self.file.write(
            "time_s,phase,pressure_mbar,temp_C,depth_m,altitude_m,"
            "pitch_deg,yaw_deg,yaw_rate_dps,yaw_target_deg\n"
        )

        self.phase = 0
        self.start_ms = time.ticks_ms()
        self.phase_start = self.start_ms
        self.current_thruster_us = self.NEUTRAL_US
        self.last_ramp_ms = self.start_ms
        self.phase6_rampdown = False
        self._reset_pid_state()
        self.last_status_ms = self.start_ms

        print("[ARM] Arming ESC at neutral")
        self.set_thruster(self.NEUTRAL_US)
        time.sleep(3)
        print("[ARM] ESC arm complete")

        self.running = True
        print("Sensor ready")
        print("Phase 0: Ballast")
        return True

    def update(self):
        if not self.running:
            return

        if self.phase == 0:
            if self._phase_time() <= 1.0:
                self.set_ballast(self.BALLAST_CLOSED)
            elif self._phase_time() <= 26.0:
                self.set_ballast(self.BALLAST_OPEN)
            else:
                self.set_ballast(self.BALLAST_CLOSED)
                print("Phase 1: Thruster Forward and Dive")
                self._next_phase()

        elif self.phase == 1:
            self._print_periodic_status(
                "[PHASE 1] Ramping thruster: {} us".format(self.current_thruster_us)
            )
            if self._ramp_thruster(self.FORWARD_US):
                print("[PHASE 1] Thruster reached forward setpoint")
                self.set_horizontal_fins_ticks(
                    self.FIN_DEFLECT_RIGHT,
                    self._mirror_fin(self.FIN_DEFLECT_RIGHT)
                )
                self._next_phase()

        elif self.phase == 2:
            self._log()
            self._print_periodic_status(
                "[PHASE 2] Diving, t={:.1f}s".format(self._phase_time())
            )
            self.set_horizontal_fins_ticks(
                self.FIN_DEFLECT_RIGHT,
                self._mirror_fin(self.FIN_DEFLECT_RIGHT)
            )
            if self._phase_time() >= 4.0:
                print("[PHASE 2] Switching dive fin bias")
                self.set_horizontal_fins_ticks(
                    self.FIN_DEFLECT_LEFT,
                    self._mirror_fin(self.FIN_DEFLECT_LEFT)
                )
                self._next_phase()

        elif self.phase == 3:
            self._log()
            self._print_periodic_status(
                "[PHASE 3] Completing dive entry, t={:.1f}s".format(self._phase_time())
            )
            self.set_horizontal_fins_ticks(
                self.FIN_DEFLECT_LEFT,
                self._mirror_fin(self.FIN_DEFLECT_LEFT)
            )
            if self._phase_time() >= 2.0:
                self.fins_neutral_ticks()
                print("Phase 4: PID Forward Hold")
                self._prepare_pid_hold()
                self._next_phase()

        elif self.phase == 4:
            self.set_thruster(self.FORWARD_US)
            self._pid_hold_heading()
            self._log()
            self._print_periodic_status(
                "[PHASE 4] Forward hold, heading={:.1f}, target={:.1f}".format(
                    self.last_yaw, self.yaw_target
                )
            )
            if self._phase_time() >= 10.0:
                print("Phase 5: PID 180 Left Turn")
                self._prepare_pid_turn()
                self._next_phase()

        elif self.phase == 5:
            self.set_thruster(self.FORWARD_US)
            self._pid_turn_180_left()
            self._log()
            self._print_periodic_status(
                "[PHASE 5] Turning, heading={:.1f}, error={:.1f}, rate={:.1f}".format(
                    self.last_yaw, self.last_yaw_error, self.last_yaw_rate
                )
            )
            if self._turn_complete():
                print("Phase 6: PID Forward Hold")
                self._prepare_pid_hold()
                self._next_phase()

        elif self.phase == 6:
            if not self.phase6_rampdown:
                self.set_thruster(self.FORWARD_US)
                self._pid_hold_heading()
                self._log()
                self._print_periodic_status(
                    "[PHASE 6] Forward hold, heading={:.1f}, target={:.1f}".format(
                        self.last_yaw, self.yaw_target
                    )
                )

                if self._phase_time() >= 10.0:
                    self.phase6_rampdown = True
                    self.last_ramp_ms = time.ticks_ms()
                    self.fins_neutral_ticks()
                    print("[PHASE 6] Hold complete, beginning rampdown")
            else:
                self._print_periodic_status(
                    "[PHASE 6] Ramping down thruster: {} us".format(self.current_thruster_us)
                )
                if self._ramp_thruster(self.TURNDOWN_US):
                    print("[PHASE 6] Rampdown complete, stopping ESC")
                    self.set_pwm(self.ESC_CH, 0)
                    self._next_phase()

        elif self.phase == 7:
            self._print_periodic_status(
                "[PHASE 7] Waiting before solenoid fire, t={:.1f}s".format(
                    self._phase_time()
                )
            )
            if self._phase_time() >= 2.0:
                print("Phase 7: Solenoid")
                self.solenoid_on()
                self._next_phase()

        elif self.phase == 8:
            self._print_periodic_status(
                "[PHASE 8] Solenoid ON, t={:.1f}s".format(self._phase_time())
            )
            if self._phase_time() >= 5.0:
                print("[PHASE 8] Solenoid OFF")
                self.solenoid_off()
                self._next_phase()

        elif self.phase == 9:
            self._print_periodic_status(
                "[PHASE 9] Pause before second solenoid fire, t={:.1f}s".format(
                    self._phase_time()
                )
            )
            if self._phase_time() >= 2.0:
                print("[PHASE 9] Solenoid ON")
                self.solenoid_on()
                self._next_phase()

        elif self.phase == 10:
            self._print_periodic_status(
                "[PHASE 10] Final solenoid ON, t={:.1f}s".format(self._phase_time())
            )
            if self._phase_time() >= 5.0:
                self.solenoid_off()
                print("Shutting down")
                self._next_phase()

        elif self.phase == 11:
            self.stop()

    def stop(self):
        if not self.running and not self.file:
            return

        print("Ballast + PID 180 Turn STOP")
        self.running = False

        try:
            if self.file:
                self.file.close()
                self.file = None
        except:
            self.file = None

        self.safe_stop_all()
        self.set_ballast(self.BALLAST_CLOSED)
        self.fins_neutral_ticks()
        self.solenoid_off()

    def fins_neutral_ticks(self):
        self.set_vertical_fins_ticks(self.FIN_NEUTRAL, self.FIN_NEUTRAL)
        self.set_horizontal_fins_ticks(self.FIN_NEUTRAL, self.FIN_NEUTRAL)

    def _pitch(self, accel):
        pitch = math.degrees(math.atan2(accel["y"], accel["z"]))
        return self.MPU_PITCH_SIGN * pitch

    def _mirror_fin(self, value):
        return self.FIN_NEUTRAL - (value - self.FIN_NEUTRAL)

    def _time(self):
        return time.ticks_diff(time.ticks_ms(), self.start_ms) / 1000

    def _phase_time(self):
        return time.ticks_diff(time.ticks_ms(), self.phase_start) / 1000

    def _next_phase(self):
        self.phase += 1
        self.phase_start = time.ticks_ms()
        self.last_status_ms = self.phase_start
        print("[STATE] Entered phase", self.phase)

    def _ramp_thruster(self, target_us):
        now = time.ticks_ms()
        if time.ticks_diff(now, self.last_ramp_ms) < self.ramp_delay_ms:
            self.set_thruster(self.current_thruster_us)
            return False

        self.last_ramp_ms = now

        if self.current_thruster_us < target_us:
            self.current_thruster_us = min(
                self.current_thruster_us + self.ramp_step,
                target_us
            )
        elif self.current_thruster_us > target_us:
            self.current_thruster_us = max(
                self.current_thruster_us - self.ramp_step,
                target_us
            )

        self.set_thruster(self.current_thruster_us)
        return self.current_thruster_us == target_us

    def _load_startup_calibration(self):
        if not has_calibration():
            print("[CAL] No startup calibration stored")
            return False

        calibration = get_calibration()
        self.pitch_offset = self.MPU_PITCH_SIGN * calibration["pitch_offset"]
        self.yaw_bias = calibration["yaw_bias"]

        print("[CAL] Loaded startup calibration")
        print("[CAL] Pitch offset:", self.pitch_offset)
        print("[CAL] Yaw bias:", self.yaw_bias)
        print(
            "[CAL] Orientation signs:",
            "pitch=", self.MPU_PITCH_SIGN,
            "yaw_rate=", self.MPU_YAW_RATE_SIGN
        )
        return True

    def _reset_pid_state(self):
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.pitch_f.reset()
        self.yaw_f.reset()
        self.yaw = 0
        self.last_t = None
        self.yaw_target = 0
        self.locked_180 = False
        self.turn_lock_start = None
        self.last_up = self.FIN_NEUTRAL
        self.last_down = self.FIN_NEUTRAL
        self.last_left = self.FIN_NEUTRAL
        self.last_right = self.FIN_NEUTRAL
        self.last_pitch = 0
        self.last_yaw = 0
        self.last_yaw_rate = 0
        self.last_yaw_error = 0

    def _prepare_pid_hold(self):
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.last_t = time.ticks_ms()
        self.yaw_target = self.last_yaw
        self.locked_180 = False
        self.turn_lock_start = None
        print("[PID] Hold target set to", round(self.yaw_target, 3), "deg")

    def _prepare_pid_turn(self):
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.last_t = time.ticks_ms()
        self.yaw_target = self.last_yaw + 180
        self.locked_180 = False
        self.turn_lock_start = None
        print("[PID] Turn target set to", round(self.yaw_target, 3), "deg")

    def _read_mpu(self):
        if self.mpu is None:
            return None

        now = time.ticks_ms()
        if self.last_t is None:
            self.last_t = now
            return None

        dt = time.ticks_diff(now, self.last_t) / 1000
        self.last_t = now
        if dt <= 0:
            return None

        try:
            accel = self.mpu.get_accel()
            gyro = self.mpu.get_gyro()
        except OSError:
            print("[WARN] MPU read failed")
            return None

        pitch = self.pitch_f.update(self._pitch(accel) - self.pitch_offset)
        yaw_rate = self.MPU_YAW_RATE_SIGN * (gyro["z"] - self.yaw_bias)
        self.yaw += yaw_rate * dt
        yaw = self.yaw_f.update(self.yaw)

        self.last_pitch = pitch
        self.last_yaw = yaw
        self.last_yaw_rate = yaw_rate
        return pitch, yaw, yaw_rate

    def _normalized_yaw_error(self, target, yaw):
        error = target - yaw
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        return error

    def _set_fins_pid(self, pitch_cmd, yaw_cmd):
        pitch_cmd *= self.fin_gain * 0.4
        yaw_cmd *= self.fin_gain * 0.4

        left = self.FIN_NEUTRAL - pitch_cmd
        right = self.FIN_NEUTRAL + pitch_cmd
        up = self.FIN_NEUTRAL - yaw_cmd
        down = self.FIN_NEUTRAL + yaw_cmd

        def clamp(x):
            return max(self.min_pwm, min(self.max_pwm, int(x)))

        self.last_left += self.alpha_out * (left - self.last_left)
        self.last_right += self.alpha_out * (right - self.last_right)
        self.last_up += self.alpha_out * (up - self.last_up)
        self.last_down += self.alpha_out * (down - self.last_down)

        self.set_pwm(self.LEFT_FIN, clamp(self.last_left))
        self.set_pwm(self.RIGHT_FIN, clamp(self.last_right))
        self.set_pwm(self.UP_FIN, clamp(self.last_up))
        self.set_pwm(self.DOWN_FIN, clamp(self.last_down))

    def _pid_hold_heading(self):
        mpu_data = self._read_mpu()
        if mpu_data is None:
            return

        pitch, yaw, yaw_rate = mpu_data
        # Hold phases need the opposite yaw correction polarity from the
        # commanded turn phase on this vehicle's fin layout.
        yaw_error = -self._normalized_yaw_error(self.yaw_target, yaw)
        yaw_cmd = self.yaw_pid.update(yaw_error) - 0.7 * yaw_rate
        pitch_cmd = self.pitch_pid.update(-pitch)

        self.last_yaw_error = yaw_error
        self._set_fins_pid(pitch_cmd, yaw_cmd)
        self._debug_pid()

    def _pid_turn_180_left(self):
        mpu_data = self._read_mpu()
        if mpu_data is None:
            return

        pitch, yaw, yaw_rate = mpu_data
        yaw_error = self._normalized_yaw_error(self.yaw_target, yaw)
        yaw_cmd = self.yaw_pid.update(yaw_error) - 0.7 * yaw_rate
        pitch_cmd = self.pitch_pid.update(-pitch)

        if not self.locked_180:
            if abs(yaw_error) < self.lock_band and abs(yaw_rate) < 8:
                self.locked_180 = True
                self.turn_lock_start = time.ticks_ms()
                self.yaw_target = yaw
                print("[LOCK] 180 achieved, holding new heading")
        else:
            if abs(yaw_error) > self.unlock_band:
                self.locked_180 = False
                self.turn_lock_start = None
                print("[LOCK] Turn lock lost, correcting again")

        self.last_yaw_error = yaw_error
        self._set_fins_pid(pitch_cmd, yaw_cmd)
        self._debug_pid()

    def _turn_complete(self):
        if not self.locked_180 or self.turn_lock_start is None:
            return False

        return (
            time.ticks_diff(time.ticks_ms(), self.turn_lock_start) / 1000
            >= self.turn_lock_hold_s
        )

    def _debug_pid(self):
        if time.ticks_ms() % 200 < 20:
            print(
                "Pitch:", round(self.last_pitch, 3),
                "| Yaw:", round(self.last_yaw, 3),
                "| Yaw error:", round(self.last_yaw_error, 3),
                "| Yaw rate:", round(self.last_yaw_rate, 3),
                "| Target:", round(self.yaw_target, 3),
                "| Locked:", self.locked_180
            )

    def _print_periodic_status(self, message, interval_ms=1000):
        now = time.ticks_ms()
        if time.ticks_diff(now, self.last_status_ms) >= interval_ms:
            print(message)
            self.last_status_ms = now

    def _log(self):
        if not self.file:
            return

        pressure, temp, depth, altitude = self.read_sensor_data()
        if pressure is None:
            return

        self.file.write(
            "{:.2f},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
                self._time(),
                self.phase,
                pressure,
                temp,
                depth,
                altitude,
                self.last_pitch,
                self.last_yaw,
                self.last_yaw_rate,
                self.yaw_target
            )
        )
