from machine import I2C
import time

class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr

        # Wake up MPU (exit sleep mode)
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
        time.sleep(0.1)

    def read_raw(self):
        data = self.i2c.readfrom_mem(self.addr, 0x3B, 14)

        def conv(h, l):
            val = (h << 8) | l
            if val > 32767:
                val -= 65536
            return val

        ax = conv(data[0], data[1])
        ay = conv(data[2], data[3])
        az = conv(data[4], data[5])

        gx = conv(data[8], data[9])
        gy = conv(data[10], data[11])
        gz = conv(data[12], data[13])

        return ax, ay, az, gx, gy, gz

    def get_accel(self):
        ax, ay, az, _, _, _ = self.read_raw()

        # scale for ±2g
        return {
            'x': ax / 16384,
            'y': ay / 16384,
            'z': az / 16384
        }

    def get_gyro(self):
        _, _, _, gx, gy, gz = self.read_raw()

        # scale for ±250 deg/s
        return {
            'x': gx / 131,
            'y': gy / 131,
            'z': gz / 131
        }