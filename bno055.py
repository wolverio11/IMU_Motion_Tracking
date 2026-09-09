from machine import I2C
import time

CHIP_ID = 0x00
UNIT_SEL = 0x3B
OPR_MODE = 0x3D
PWR_MODE = 0x3E
SYS_TRIGGER = 0x3F
CALIB_STAT = 0x35

ACCEL = 0x08
MAG = 0x0E
GYRO = 0x14
EULER = 0x1A
QUAT = 0x20
LINEAR = 0x28
GRAV = 0x2E
TEMP = 0x34


class BNO055:

    def __init__(self, i2c, addr=0x28):
        self.i2c = i2c
        self.addr = addr
        self.init()

    def s16(self, lsb, msb):
        v = lsb | (msb << 8)
        if v >= 32768:
            v -= 65536
        return v

    def _vec(self, reg, scale):
        d = self.i2c.readfrom_mem(self.addr, reg, 6)

        return (
            self.s16(d[0], d[1]) / scale,
            self.s16(d[2], d[3]) / scale,
            self.s16(d[4], d[5]) / scale,
        )

    def _quat(self):
        d = self.i2c.readfrom_mem(self.addr, QUAT, 8)

        return (
            self.s16(d[0], d[1]) / 16384.0,
            self.s16(d[2], d[3]) / 16384.0,
            self.s16(d[4], d[5]) / 16384.0,
            self.s16(d[6], d[7]) / 16384.0,
        )

    def init(self):

        chip = self.i2c.readfrom_mem(self.addr, CHIP_ID, 1)[0]
        if chip != 0xA0:
            raise Exception("BNO055 not found")

        self.i2c.writeto_mem(self.addr, SYS_TRIGGER, b'\x20')
        time.sleep(1)

        while True:
            try:
                if self.i2c.readfrom_mem(self.addr, CHIP_ID, 1)[0] == 0xA0:
                    break
            except:
                pass

            time.sleep_ms(10)

        self.i2c.writeto_mem(self.addr, OPR_MODE, b'\x00')
        time.sleep_ms(30)

        self.i2c.writeto_mem(self.addr, PWR_MODE, b'\x00')
        time.sleep_ms(20)

        self.i2c.writeto_mem(self.addr, UNIT_SEL, b'\x00')
        time.sleep_ms(20)

        self.i2c.writeto_mem(self.addr, SYS_TRIGGER, b'\x00')
        time.sleep_ms(20)

        self.i2c.writeto_mem(self.addr, OPR_MODE, b'\x0C')
        time.sleep_ms(50)

    def accel(self):
        return self._vec(ACCEL, 100)

    def gyro(self):
        return self._vec(GYRO, 16)

    def mag(self):
        return self._vec(MAG, 16)

    def euler(self):
        return self._vec(EULER, 16)

    def linear(self):
        return self._vec(LINEAR, 100)

    def gravity(self):
        return self._vec(GRAV, 100)

    def quaternion(self):
        return self._quat()

    def temperature(self):
        return self.i2c.readfrom_mem(self.addr, TEMP, 1)[0]

    def calibration(self):
        c = self.i2c.readfrom_mem(self.addr, CALIB_STAT, 1)[0]

        return (
            (c >> 6) & 3,
            (c >> 4) & 3,
            (c >> 2) & 3,
            c & 3,
        )
