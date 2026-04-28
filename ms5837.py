from machine import I2C
import time

# Models
MODEL_02BA = 0
MODEL_30BA = 1

# Oversampling
OSR_256  = 0
OSR_512  = 1
OSR_1024 = 2
OSR_2048 = 3
OSR_4096 = 4
OSR_8192 = 5

# Conversion time (seconds)
OSR_DELAY = [0.001, 0.002, 0.003, 0.005, 0.010, 0.020]

# Commands
RESET = 0x1E
ADC_READ = 0x00
PROM_READ = 0xA0
CONVERT_D1 = 0x40
CONVERT_D2 = 0x50


class MS5837:
    def __init__(self, i2c, model=MODEL_30BA, address=0x76):
        self._i2c = i2c
        self._model = model
        self._address = address

        self._C = [0]*7
        self._D1 = 0
        self._D2 = 0

        self._pressure = 0
        self._temperature = 0

        self._fluidDensity = 997  # kg/m^3

    def init(self):
        try:
            self._reset()
            time.sleep(0.02)

            for i in range(7):
                self._C[i] = self._read_prom(i)
#CRC check disabled for MicroPython compatibility
#            if not self._crc4(self._C):
#                print("CRC failed")
#                return False

            return True
        except:
            return False

    def _reset(self):
        self._i2c.writeto(self._address, bytes([RESET]))

    def _read_prom(self, index):
        cmd = PROM_READ + index * 2
        self._i2c.writeto(self._address, bytes([cmd]))
        data = self._i2c.readfrom(self._address, 2)
        return data[0] << 8 | data[1]

    def _convert(self, cmd, osr):
        self._i2c.writeto(self._address, bytes([cmd + (osr * 2)]))
        time.sleep(OSR_DELAY[osr])

        self._i2c.writeto(self._address, bytes([ADC_READ]))
        data = self._i2c.readfrom(self._address, 3)
        return data[0] << 16 | data[1] << 8 | data[2]

    def read(self, osr=OSR_4096):
        try:
            self._D1 = self._convert(CONVERT_D1, osr)
            self._D2 = self._convert(CONVERT_D2, osr)

            dT = self._D2 - self._C[5] * 256

            if self._model == MODEL_02BA:
                SENS = self._C[1] * 65536 + (self._C[3] * dT) / 128
                OFF  = self._C[2] * 131072 + (self._C[4] * dT) / 64
                TEMP = 2000 + dT * self._C[6] / 8388608
                P = (self._D1 * SENS / 2097152 - OFF) / 32768
            else:
                SENS = self._C[1] * 32768 + (self._C[3] * dT) / 256
                OFF  = self._C[2] * 65536 + (self._C[4] * dT) / 128
                TEMP = 2000 + dT * self._C[6] / 8388608
                P = (self._D1 * SENS / 2097152 - OFF) / 8192

            # Second order compensation
            Ti = 0
            OFFi = 0
            SENSi = 0

            if TEMP < 2000:
                Ti = (3 * dT * dT) / 8589934592
                OFFi = (3 * (TEMP - 2000)**2) / 2
                SENSi = (5 * (TEMP - 2000)**2) / 8

                if TEMP < -1500:
                    OFFi += 7 * (TEMP + 1500)**2
                    SENSi += 4 * (TEMP + 1500)**2

            TEMP -= Ti
            OFF -= OFFi
            SENS -= SENSi

            if self._model == MODEL_02BA:
                P = (self._D1 * SENS / 2097152 - OFF) / 32768
            else:
                P = (self._D1 * SENS / 2097152 - OFF) / 8192

            self._temperature = TEMP / 100.0
            self._pressure = P / 100.0  # mbar

            return True

        except:
            return False

    def pressure(self):
        return self._pressure

    def temperature(self):
        return self._temperature

    def depth(self, surface_pressure=None):
        if surface_pressure is None:
            surface_pressure = 1013.25  # fallback sea-level
        return (self._pressure - surface_pressure) * 100 / (self._fluidDensity * 9.80665)
    
    def altitude(self):
        return (1 - (self._pressure / 1013.25)**0.190284) * 145366.45 * 0.3048

    def setFluidDensity(self, density):
        self._fluidDensity = density

    def air_density(self):
        pressure_pa = self._pressure * 100
        temp_k = self._temperature + 273.15
        return pressure_pa / (287.05 * temp_k)

    # CRC check (unchanged logic)
    def _crc4(self, n_prom):
        n_rem = 0
        n_prom[0] = n_prom[0] & 0x0FFF
        n_prom.append(0)

        for i in range(16):
            if i % 2 == 1:
                n_rem ^= n_prom[i >> 1] & 0x00FF
            else:
                n_rem ^= n_prom[i >> 1] >> 8

            for _ in range(8):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem = n_rem << 1

        n_rem = (n_rem >> 12) & 0xF
        return n_rem == (n_prom[0] >> 12)


class MS5837_30BA(MS5837):
    def __init__(self, i2c, address=0x76):
        super().__init__(i2c, MODEL_30BA, address)


class MS5837_02BA(MS5837):
    def __init__(self, i2c, address=0x76):
        super().__init__(i2c, MODEL_02BA, address)