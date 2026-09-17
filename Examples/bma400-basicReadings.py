# FILE: bma400-basicReadings.py
# AUTHOR: Soldered Electronics
# BRIEF: Reads X, Y, Z acceleration from the BMA400 sensor over I2C
# WORKS WITH: BMA400 Accelerometer breakout: www.solde.red/333419
# LAST UPDATED: 2026-09-17

from bma400 import BMA400, BMA400_OK
from machine import I2C, Pin
import time

# Change these to match how your board wires I2C
i2c = I2C(0, scl=Pin(9), sda=Pin(8))
sensor = BMA400(i2c)

if sensor.status != BMA400_OK:
    raise Exception("Failed to initialize BMA400: " + sensor.status_string())

print("BMA400 connected!")

while True:
    if sensor.get_sensor_data() == BMA400_OK:
        print(
            "X: {:.3f} g\tY: {:.3f} g\tZ: {:.3f} g".format(
                sensor.data.accel_x, sensor.data.accel_y, sensor.data.accel_z
            )
        )
    else:
        print("Failed to read acceleration:", sensor.status_string())

    time.sleep_ms(20)
