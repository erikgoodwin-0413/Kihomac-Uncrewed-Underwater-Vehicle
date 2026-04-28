import time
from tests.base_test import SubTestBase


class BallastTest30(SubTestBase):
    name = "Ballast Test 3.0"

    def __init__(self):
        super().__init__()

        # Thruster PWM in microseconds
        self.NEUTRAL_US = 1500
        self.FORWARD_US = 1800
        self.TURNDOWN_US = 1600

        # Ballast servo angles
        self.BALLAST_CLOSED = 180
        self.BALLAST_OPEN = 0

        # Raw PCA9685 tick values from the original script
        self.FIN_NEUTRAL = 307
        self.FIN_DEFLECT_RIGHT = 360
        self.FIN_DEFLECT_LEFT = 254

        # State
        self.phase = 0
        self.phase_start = 0
        self.start_ms = 0
        self.file = None
        self.current_thruster_us = self.NEUTRAL_US
        self.last_ramp_ms = 0

        # Ramp tuning from the original script
        self.ramp_step = 10
        self.ramp_delay_ms = 100

    def start(self):
        print("Ballast Test 3.0 START")

        if not self.init_hardware():
            print("Hardware init failed")
            return False

        sensor_ready = self.sensor.init()
        if not sensor_ready:
            print("Sensor init failed")
            return False

        self.sensor.setFluidDensity(1.0108099)
        self.safe_stop_all()

        self.solenoid_off()
        self.fins_neutral()
        self.set_ballast(self.BALLAST_CLOSED)

        self.file = open("pressure_log.csv", "w")
        self.file.write("time_s,pressure_mbar,temp_C,depth_m,altitude_m\n")

        self.phase = 0
        self.start_ms = time.ticks_ms()
        self.phase_start = self.start_ms
        self.current_thruster_us = self.NEUTRAL_US
        self.last_ramp_ms = self.start_ms

        # Keep ESC arming in start so the mission begins from a known safe state.
        self.set_thruster(self.NEUTRAL_US)
        time.sleep(3)

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
            if self._ramp_thruster(self.FORWARD_US):
                self.set_horizontal_fins_ticks(
                    self.FIN_DEFLECT_RIGHT,
                    self._mirror_fin(self.FIN_DEFLECT_RIGHT)
                )
                self._next_phase()

        elif self.phase == 2:
            self._log()
            self.set_horizontal_fins_ticks(
                self.FIN_DEFLECT_RIGHT,
                self._mirror_fin(self.FIN_DEFLECT_RIGHT)
            )
            if self._phase_time() >= 4.0:
                self.set_horizontal_fins_ticks(
                    self.FIN_DEFLECT_LEFT,
                    self._mirror_fin(self.FIN_DEFLECT_LEFT)
                )
                self._next_phase()

        elif self.phase == 3:
            self._log()
            self.set_horizontal_fins_ticks(
                self.FIN_DEFLECT_LEFT,
                self._mirror_fin(self.FIN_DEFLECT_LEFT)
            )
            if self._phase_time() >= 2.0:
                self.fins_neutral_ticks()
                print("Phase 2: Thruster Coast")
                self._next_phase()

        elif self.phase == 4:
            self._log()
            if self._phase_time() >= 5.0:
                self.set_vertical_fins_ticks(self.FIN_DEFLECT_RIGHT, self.FIN_NEUTRAL)
                self._next_phase()

        elif self.phase == 5:
            self._log()
            self.set_vertical_fins_ticks(self.FIN_DEFLECT_RIGHT, self.FIN_NEUTRAL)
            if self._phase_time() >= 10.0:
                self.fins_neutral_ticks()
                self._next_phase()

        elif self.phase == 6:
            if self._ramp_thruster(self.TURNDOWN_US):
                self.set_pwm(self.ESC_CH, 0)
                self._next_phase()

        elif self.phase == 7:
            if self._phase_time() >= 2.0:
                print("Phase 3: Solenoid")
                self.solenoid_on()
                self._next_phase()

        elif self.phase == 8:
            if self._phase_time() >= 5.0:
                self.solenoid_off()
                self._next_phase()

        elif self.phase == 9:
            if self._phase_time() >= 2.0:
                self.solenoid_on()
                self._next_phase()

        elif self.phase == 10:
            if self._phase_time() >= 5.0:
                self.solenoid_off()
                print("Shutting down")
                self._next_phase()

        elif self.phase == 11:
            self.stop()

    def stop(self):
        if not self.running and not self.file:
            return

        print("Ballast Test 3.0 STOP")
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

    def _mirror_fin(self, value):
        return self.FIN_NEUTRAL - (value - self.FIN_NEUTRAL)

    def _time(self):
        return time.ticks_diff(time.ticks_ms(), self.start_ms) / 1000

    def _phase_time(self):
        return time.ticks_diff(time.ticks_ms(), self.phase_start) / 1000

    def _next_phase(self):
        self.phase += 1
        self.phase_start = time.ticks_ms()

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

    def _log(self):
        if not self.file:
            return

        pressure, temp, depth, altitude = self.read_sensor_data()
        if pressure is None:
            return

        self.file.write(
            "{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
                self._time(),
                pressure,
                temp,
                depth,
                altitude
            )
        )
