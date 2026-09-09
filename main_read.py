from machine import Pin, I2C, UART
from bno055 import BNO055

import network
import espnow
import time
import esp32

# =====================================================
# UART
# =====================================================

uart = UART(0, baudrate=115200)

CMD = "#cmdareyouthere#end"

rx_buffer = ""

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

# =====================================================

PERIOD_US = 10000
DOCK_TIMEOUT = 500     # ms

# =====================================================


def uart_command_received():
    global rx_buffer

    if uart.any():

        d = uart.read()

        if d:

            try:
                rx_buffer += d.decode()
            except:
                return False

            if len(rx_buffer) > 128:
                rx_buffer = rx_buffer[-128:]

            if CMD in rx_buffer:
                rx_buffer = ""
                return True

    return False


# =====================================================

def docked_mode():

    print("Entered DOCKED mode")

    last_cmd = time.ticks_ms()

    while True:

        if uart_command_received():
            last_cmd = time.ticks_ms()

        if time.ticks_diff(time.ticks_ms(), last_cmd) > DOCK_TIMEOUT:
            print("Dock timeout")
            return

        # -------------------------------------------------
        # Future dock commands go here
        # -------------------------------------------------


# =====================================================

def undocked_mode():

    print("Entered UNDOCKED mode")

    next_time = time.ticks_us()

    packets = 0
    last = time.ticks_ms()

    while True:

        if uart_command_received():
            print("Dock request")
            return

        accel = imu.i2c.readfrom_mem(imu.addr, 0x08, 6)
        mag   = imu.i2c.readfrom_mem(imu.addr, 0x0E, 6)
        gyro  = imu.i2c.readfrom_mem(imu.addr, 0x14, 6)
        euler = imu.i2c.readfrom_mem(imu.addr, 0x1A, 6)
        quat  = imu.i2c.readfrom_mem(imu.addr, 0x20, 8)

        calib = imu.i2c.readfrom_mem(imu.addr, 0x35, 1)
        temp  = imu.i2c.readfrom_mem(imu.addr, 0x34, 1)

        payload = accel + gyro + mag + euler + quat + calib + temp

        e.send(peer, payload, False)

        packets += 1

        if time.ticks_diff(time.ticks_ms(), last) >= 1000:

            print("Packets/sec :", packets)
            print("MCU Temp    :", esp32.mcu_temperature())

            packets = 0
            last = time.ticks_ms()

        next_time = time.ticks_add(next_time, PERIOD_US)

        while time.ticks_diff(next_time, time.ticks_us()) > 0:
            pass


# =====================================================
# Startup
# =====================================================

print("Waiting 2 seconds for docking request...")

start = time.ticks_ms()

docked = False

while time.ticks_diff(time.ticks_ms(), start) < 2000:

    if uart_command_received():
        docked = True
        break

while True:

    if docked:
        docked_mode()
        docked = False

    else:
        undocked_mode()
        docked = True
