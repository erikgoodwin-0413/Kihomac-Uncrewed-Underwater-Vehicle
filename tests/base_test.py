from machine import I2C, Pin
import time
import ms5837


class SubTestBase:
    """
    Shared hardware + safety + sensor layer for all submarine tests.
    """

    # PWM conversion constant (50Hz)
    TICK_US = 4.88
    PWM_MIN_US = 1000
    PWM_NEUTRAL_US = 1500
    PWM_MAX_US = 2000

    def __init__(self):
        self.running = False

        # I2C / devices
        self.i2c = None
        self.sensor = None

        # Hardware pins
        self.led = Pin(14, Pin.OUT)
        self.relay = Pin(13, Pin.OUT)

        # PCA9685
        self.PCA_ADDR = 0x40

        # Channels
        self.LEFT_FIN = 0
        self.RIGHT_FIN = 1
        self.UP_FIN = 2
        self.DOWN_FIN = 3
        self.BALLAST = 4
        self.ESC_CH = 5

        # Depth calibration
        self.depth_offset = 0

    # -----------------------
    # PCA9685 INITIALIZATION
    # -----------------------
    def init_pca(self):
        # Wake
        self.i2c.writeto_mem(self.PCA_ADDR, 0x00, bytes([0x00]))
        time.sleep(0.01)

        # Enter sleep to set prescaler
        self.i2c.writeto_mem(self.PCA_ADDR, 0x00, bytes([0x10]))

        # Set prescaler for 50 Hz
        self.i2c.writeto_mem(self.PCA_ADDR, 0xFE, bytes([121]))

        # Wake + auto-increment
        self.i2c.writeto_mem(self.PCA_ADDR, 0x00, bytes([0x20]))
        time.sleep(0.01)

        # Restart
        self.i2c.writeto_mem(self.PCA_ADDR, 0x00, bytes([0xA1]))
        time.sleep(0.01)

    # -----------------------
    # PWM CORE
    # -----------------------
    def us_to_ticks(self, us):
        return int(us / self.TICK_US)

    def angle_to_ticks(self, angle):
        us = self.PWM_MIN_US + (angle / 180.0) * (self.PWM_MAX_US - self.PWM_MIN_US)
        return self.us_to_ticks(us)

    def set_pwm(self, channel, value):
        on = 0
        off = int(value)
        reg = 0x06 + 4 * channel

        self.i2c.writeto_mem(
            self.PCA_ADDR,
            reg,
            bytes([on & 0xFF, on >> 8, off & 0xFF, off >> 8])
        )

    # -----------------------
    # ACTUATORS
    # -----------------------
    def set_thruster(self, us):
        self.set_pwm(self.ESC_CH, self.us_to_ticks(us))

    def fins_neutral(self, us=1500):
        pwm = self.us_to_ticks(us)
        self.set_pwm(self.LEFT_FIN, pwm)
        self.set_pwm(self.RIGHT_FIN, pwm)
        self.set_pwm(self.UP_FIN, pwm)
        self.set_pwm(self.DOWN_FIN, pwm)

    def set_horizontal_fins_ticks(self, left, right):
        self.set_pwm(self.LEFT_FIN, left)
        self.set_pwm(self.RIGHT_FIN, right)

    def set_vertical_fins_ticks(self, up, down):
        self.set_pwm(self.UP_FIN, up)
        self.set_pwm(self.DOWN_FIN, down)

    def set_horizontal_fins_us(self, left_us, right_us):
        self.set_horizontal_fins_ticks(
            self.us_to_ticks(left_us),
            self.us_to_ticks(right_us)
        )

    def set_vertical_fins_us(self, up_us, down_us):
        self.set_vertical_fins_ticks(
            self.us_to_ticks(up_us),
            self.us_to_ticks(down_us)
        )

    def set_ballast(self, angle):
        # Convert 0–180° → 1000–2000 µs
        us = 1000 + (angle / 180.0) * 1000
        self.set_pwm(self.BALLAST, self.us_to_ticks(us))

    def solenoid_on(self):
        self.relay.value(1)

    def solenoid_off(self):
        self.relay.value(0)

    # -----------------------
    # SENSOR
    # -----------------------
    def read_depth(self):
        if not self.sensor or not self.sensor.read():
            return None
        return self.sensor.depth() - self.depth_offset

    def depth_avg(self, n=3):
        vals = []
        for _ in range(n):
            d = self.read_depth()
            if d is not None:
                vals.append(d)
            time.sleep(0.03)

        return sum(vals) / len(vals) if vals else None

    def calibrate_surface(self):
        vals = []
        for _ in range(10):
            if self.sensor.read():
                vals.append(self.sensor.depth())
            time.sleep(0.1)

        if vals:
            self.depth_offset = sum(vals) / len(vals)

    # -----------------------
    # INITIALIZATION
    # -----------------------
    def init_hardware(self):
        # Initialize I2C
        if self.i2c is None:
            self.i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)

            # Initialize PCA9685
            self.init_pca()

        # Initialize pressure sensor
        self.sensor = ms5837.MS5837_02BA(self.i2c)
        self.sensor.setFluidDensity(997)

        #if not self.sensor.init():
        #    print("WARNING: Sensor init failed")
        #    return False

        return True

    def init_sensors(self):
        return self.init_hardware()

    def read_sensor_data(self):
        if not self.sensor or not self.sensor.read():
            return None, None, None, None

        return (
            self.sensor.pressure(),
            self.sensor.temperature(),
            self.sensor.depth(),
            self.sensor.altitude()
        )

    def arm_esc(self):
        print("Arming ESC...")
        self.set_thruster(1500)
        time.sleep(3)

    # -----------------------
    # SAFETY
    # -----------------------
    def safe_stop_all(self):
        try:
            self.set_thruster(1500)
            self.fins_neutral()
            self.set_ballast(0)
            self.solenoid_off()

            # Ensure ESC fully stops
            time.sleep(0.2)
            self.set_pwm(self.ESC_CH, 0)

        except:
            pass

        self.led.value(0)
