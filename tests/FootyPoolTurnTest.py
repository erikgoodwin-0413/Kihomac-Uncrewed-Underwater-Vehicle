import time
from tests.base_test import SubTestBase


class FootyPoolTurnTest(SubTestBase):
    name = "Turn Test"

    def __init__(self):
        super().__init__()

        # -----------------------
        # STATE MACHINE
        # -----------------------
        self.phase = 0
        self.phase_start = 0
        self.start_ms = 0

        # -----------------------
        # PARAMETERS
        # -----------------------
        self.NEUTRAL = 1500
        self.FORWARD = 1800   # ~90% thrust

        self.FIN_NEUTRAL = 1500
        self.FIN_DELTA = 250
        self.ramp_step = 10
        self.ramp_delay = 0.05

        # durations (seconds)
        self.forward_time_1 = 5
        self.turn_time = 10
        self.forward_time_2 = 5

        # logging
        self.file = None

    # -----------------------
    # START
    # -----------------------
    def start(self):
        print("Footy Pool Turn Test START")

        if not self.init_hardware():
            print("Sensor init failed (continuing anyway)")

        self.set_thruster(self.NEUTRAL)
        self.fins_neutral()
        self.solenoid_off()

        print("Arming ESC (stable neutral)...")
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 3000:
            self.set_thruster(self.NEUTRAL)
            time.sleep(0.02)
        print("ESC armed")

        self.phase = 0
        self.start_ms = time.ticks_ms()
        self.phase_start = self.start_ms

        # logging
        self.file = open("pressure_log.csv", "w")
        self.file.write("time_s,pressure,temp,depth,alt\n")

        self.running = True

        # initial state
        self.set_thruster(self.NEUTRAL)
        self.fins_neutral()

        return True

    # -----------------------
    # UPDATE LOOP
    # -----------------------
    def update(self):
        if not self.running:
            return

        # -----------------------
        # PHASE 0: FORWARD
        # -----------------------
        if self.phase == 0:
            if self._ramp(self.NEUTRAL, self.FORWARD):
                print("Forward thrust reached")
                self._next_phase()
            self._log()

        # -----------------------
        # PHASE 1: FORWARD
        # -----------------------
        elif self.phase == 1:
            self.set_thruster(self.FORWARD)
            self._log()

            if self._phase_time() > self.forward_time_1:
                self._next_phase()

        # -----------------------
        # PHASE 2: TURN
        # -----------------------
        elif self.phase == 2:
            self.set_thruster(self.FORWARD)

            # vertical fin deflection → turning
            self.set_vertical_fins_us(
                self.FIN_NEUTRAL + self.FIN_DELTA,
                self.FIN_NEUTRAL - self.FIN_DELTA
            )

            self._log()

            if self._phase_time() > self.turn_time:
                self.fins_neutral()
                self._next_phase()

        # -----------------------
        # PHASE 3: FORWARD AGAIN
        # -----------------------
        elif self.phase == 3:
            self.set_thruster(self.FORWARD)
            self._log()

            if self._phase_time() > self.forward_time_2:
                self._next_phase()

        # -----------------------
        # PHASE 4: COMPLETE
        # -----------------------
        elif self.phase == 4:
            print("Footy Pool Turn Test COMPLETE")
            self.running = False

    # -----------------------
    # STOP (CRITICAL SAFETY)
    # -----------------------
    def stop(self):
        print("Footy Pool Turn Test STOP")

        self.running = False

        try:
            self.safe_stop_all()

            if self.file:
                self.file.close()

        except:
            pass

        self.led.value(0)

    # -----------------------
    # HELPERS
    # -----------------------
    def _phase_time(self):
        return time.ticks_diff(time.ticks_ms(), self.phase_start) / 1000

    def _next_phase(self):
        self.phase += 1
        self.phase_start = time.ticks_ms()
        print("Phase ->", self.phase)

    def _time(self):
        return time.ticks_diff(time.ticks_ms(), self.start_ms) / 1000

    def _ramp(self, start_us, end_us):
        if not hasattr(self, "_ramp_value"):
            self._ramp_value = start_us

        if start_us < end_us:
            self._ramp_value += self.ramp_step
            if self._ramp_value >= end_us:
                self._ramp_value = end_us
                self.set_thruster(end_us)
                self._ramp_value = start_us
                return True
        else:
            self._ramp_value -= self.ramp_step
            if self._ramp_value <= end_us:
                self._ramp_value = end_us
                self.set_thruster(end_us)
                self._ramp_value = start_us
                return True

        self.set_thruster(self._ramp_value)
        time.sleep(self.ramp_delay)
        return False

    def _log(self):
        if not self.file:
            return

        if self.sensor and self.sensor.read():
            P = self.sensor.pressure()
            T = self.sensor.temperature()
            D = self.sensor.depth()
            A = self.sensor.altitude()

            self.file.write("{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
                self._time(), P, T, D, A
            ))
