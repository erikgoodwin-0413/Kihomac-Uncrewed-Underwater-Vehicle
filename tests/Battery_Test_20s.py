import time
from tests.base_test import SubTestBase


class SpeedTest20s(SubTestBase):
    name = "Speed Test 20s"

    def __init__(self):
        super().__init__()

        # -----------------------
        # STATE MACHINE
        # -----------------------
        self.phase = 0
        self.start_ms = 0
        self.phase_start = 0

        # -----------------------
        # PARAMETERS
        # -----------------------
        self.FORWARD_US = 1800
        self.NEUTRAL_US = 1500

        self.test_duration = 20  # 20 s

        # ramp settings
        self.ramp_step = 10
        self.ramp_delay = 0.05

        self.file = None

    # -----------------------
    # START
    # -----------------------
    def start(self):
        print("Battery Life Test START")

        if not self.init_hardware():
            print("Sensor init failed (continuing anyway)")
        self.arm_esc()
        self.safe_stop_all()

        self.phase = 0
        self.start_ms = time.ticks_ms()
        self.phase_start = self.start_ms

        # open log
        self.file = open("battery_log.csv", "w")
        self.file.write("time_s,depth\n")

        self.running = True

        # arm ESC safely
        self.set_thruster(self.NEUTRAL_US)
        time.sleep(2)

        return True

    # -----------------------
    # UPDATE LOOP
    # -----------------------
    def update(self):
        if not self.running:
            return

        t = self._time()

        # -----------------------
        # PHASE 0: RAMP UP THRUSTER
        # -----------------------
        if self.phase == 0:
            if self._ramp(self.NEUTRAL_US, self.FORWARD_US):
                self._next_phase()

        # -----------------------
        # PHASE 1: FULL THRUST RUN
        # -----------------------
        elif self.phase == 1:
            self.set_thruster(self.FORWARD_US)
            self._log()

            if t > self.test_duration:
                self._next_phase()

        # -----------------------
        # PHASE 2: RAMP DOWN
        # -----------------------
        elif self.phase == 2:
            if self._ramp(self.FORWARD_US, self.NEUTRAL_US):
                self._next_phase()

        # -----------------------
        # PHASE 3: FINAL SAFE STOP
        # -----------------------
        elif self.phase == 3:
            self.set_thruster(self.NEUTRAL_US)
            time.sleep(0.5)

            self.set_thruster(1500)
            self.phase = 4

        # -----------------------
        # DONE
        # -----------------------
        elif self.phase == 4:
            print("Battery Test COMPLETE")
            self.running = False

    # -----------------------
    # STOP (SAFETY CRITICAL)
    # -----------------------
    def stop(self):
        print("Battery Test STOP")

        self.running = False

        try:
            self.set_thruster(1500)
            self.fins_neutral()

            if self.file:
                self.file.close()

        except:
            pass

        self.safe_stop_all()
        self.led.value(0)

    # -----------------------
    # HELPERS
    # -----------------------
    def _time(self):
        return time.ticks_diff(time.ticks_ms(), self.start_ms) / 1000

    def _next_phase(self):
        self.phase += 1
        self.phase_start = time.ticks_ms()
        print("Phase ->", self.phase)

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

        depth = self.read_depth()
        t = self._time()

        if depth is None:
            depth = 0

        self.file.write("{:.2f},{:.3f}\n".format(t, depth))
