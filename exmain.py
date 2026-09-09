from machine import Pin, I2C
from bno055 import BNO055

import network
import espnow
import time
import esp32

# =====================================================
# WiFi / ESP-NOW
# =====================================================

sta = network.WLAN(network.WLAN.IF_STA)
sta.active(True)
sta.disconnect()

e = espnow.ESPNow()
e.active(True)

peer = b'\xe0\x8c\xfeX\xd0\x8c'
e.add_peer(peer)

sta.config(txpower=6)

# =====================================================
# IMU
# =====================================================

i2c = I2C(
    0,
    scl=Pin(6),
    sda=Pin(5),
    freq=400000
)

imu = BNO055(i2c)

print("Running...")

PERIOD_US = 10000      # 100 Hz
next_time = time.ticks_us()

packets = 0
last = time.ticks_ms()

while True:
    # Read raw registers directly
    accel = imu.i2c.readfrom_mem(imu.addr, 0x08, 6)
    mag   = imu.i2c.readfrom_mem(imu.addr, 0x0E, 6)
    gyro  = imu.i2c.readfrom_mem(imu.addr, 0x14, 6)
    euler = imu.i2c.readfrom_mem(imu.addr, 0x1A, 6)
    quat  = imu.i2c.readfrom_mem(imu.addr, 0x20, 8)
    calib = imu.i2c.readfrom_mem(imu.addr, 0x35, 1)
    temp  = imu.i2c.readfrom_mem(imu.addr, 0x34, 1)

    payload = (
        accel +
        gyro +
        mag +
        euler +
        quat +
        calib +
        temp
    )

    e.send(peer, payload, False)

    packets += 1

    if time.ticks_diff(time.ticks_ms(), last) >= 1000:
        print("Packets/sec:", packets)
        print("MCU Temp:", esp32.mcu_temperature())

        packets = 0
        last = time.ticks_ms()

    next_time = time.ticks_add(next_time, PERIOD_US)

    while time.ticks_diff(next_time, time.ticks_us()) > 0:
        pass