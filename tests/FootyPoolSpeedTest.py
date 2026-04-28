import time
from tests.base_test import SubTestBase


class FootyPoolSpeedTest(SubTestBase):
    name = "Speed Test"

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
        self.FORWARD = 1800

        self.run_time = 20  # seconds

        # logging
        self.file = None

    # -----------------------
    # START
    # -----------------------
    def start(self):
        print("Footy Pool Speed Test START")

        # Initialize hardware (safe to call each test)
        if not self.init_hardware():
            print("Sensor init failed (continuing anyway)")

        # Reset actuators WITHOUT killing ESC signal
        self.set_thruster(self.NEUTRAL)
        self.fins_neutral()
        self.solenoid_off()

        # 🔥 CRITICAL: clean ESC arm with continuous neutral
        print("Arming ESC (stable neutral)...")
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 3000:
            self.set_thruster(self.NEUTRAL)
            time.sleep(0.02)

        print("ESC armed")

        # Init timing
        self.phase = 0
        self.start_ms = time.ticks_ms()
        self.phase_start = self.start_ms

        # logging
        self.file = open("pressure_log.csv", "w")
        self.file.write("time_s,pressure,temp,depth,alt\n")

        self.running = True
        return True

    # -----------------------
    # UPDATE LOOP
    # -----------------------
    def update(self):
        if not self.running:
            return

        # -----------------------
        # PHASE 0: START THRUST
        # -----------------------
        if self.phase == 0:
            print("Phase 0: Forward thrust (stabilizing)")

            # hold forward with continuous updates for 1–2 seconds
            start = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), start) < 1500:
                self.set_thruster(self.FORWARD)
                time.sleep(0.02)

            self._next_phase()
        # -----------------------
        # PHASE 1: RUN + LOG
        # -----------------------
        elif self.phase == 1:
            self.set_thruster(self.FORWARD)
            self._log()

            if self._phase_time() > self.run_time:
                self._next_phase()

        # -----------------------
        # PHASE 2: COMPLETE
        # -----------------------
        elif self.phase == 2:
            print("Speed Test COMPLETE")
            self.running = False

    # -----------------------
    # STOP (SAFE)
    # -----------------------
    def stop(self):
        print("Footy Pool Speed Test STOP")

        self.running = False

        try:
            # 🔥 DO NOT kill PWM completely → keep ESC happy
            self.set_thruster(self.NEUTRAL)
            self.fins_neutral()
            self.solenoid_off()

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