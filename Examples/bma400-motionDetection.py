# FILE: bma400-motionDetection.py
# AUTHOR: Soldered Electronics
# BRIEF: Uses the BMA400's generic interrupt feature to detect motion via a
#        hardware interrupt pin
# WORKS WITH: BMA400 Accelerometer breakout: www.solde.red/333419
# LAST UPDATED: 2026-09-17

from machine import I2C, Pin
from bma400 import (
    BMA400,
    BMA400_OK,
    BMA400_AXIS_XYZ_EN,
    BMA400_DATA_SRC_ACCEL_FILT_2,
    BMA400_ACTIVITY_INT,
    BMA400_ANY_AXES_INT,
    BMA400_UPDATE_EVERY_TIME,
    BMA400_HYST_96_MG,
    BMA400_INT_CHANNEL_1,
    BMA400_INT_PUSH_PULL_ACTIVE_1,
    BMA400_GEN1_INT_EN,
    BMA400_ASSERTED_GEN1_INT,
)

# Hardware setup: connect the breakout board's INT1 pin to INT1_PIN below
INT1_PIN = 5

mems_event = False


def int1_callback(pin):
    global mems_event
    mems_event = True


# Change these to match how your board wires I2C
i2c = I2C(0, scl=Pin(9), sda=Pin(8))
sensor = BMA400(i2c)

if sensor.status != BMA400_OK:
    raise Exception("Failed to initialize BMA400: " + sensor.status_string())

# Configure the generic interrupt feature. It triggers an interrupt when the
# measured acceleration exceeds a reference acceleration
config = {
    "gen_int_thres": 5,  # 8mg resolution (e.g. 5 = 40mg)
    "gen_int_dur": 5,  # 10ms resolution (e.g. 5 = 50ms)
    "axes_sel": BMA400_AXIS_XYZ_EN,  # Evaluate all axes for interrupts
    "data_src": BMA400_DATA_SRC_ACCEL_FILT_2,  # Datasheet recommends filter 2 (100Hz)
    "criterion_sel": BMA400_ACTIVITY_INT,  # Trigger when active
    "evaluate_axes": BMA400_ANY_AXES_INT,  # Trigger if any axis exceeds threshold
    "ref_update": BMA400_UPDATE_EVERY_TIME,  # Automatically update reference values
    "hysteresis": BMA400_HYST_96_MG,  # Hysteresis for noise rejection
    "int_thres_ref_x": 0,
    "int_thres_ref_y": 0,
    "int_thres_ref_z": 512,  # 1g at 4g range
    "int_chan": BMA400_INT_CHANNEL_1,  # Route interrupt to INT1 pin
}
sensor.set_generic1_interrupt(config)

# Configure INT1 pin to push/pull mode, active high
sensor.set_interrupt_pin_mode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1)

# Enable the generic 1 interrupt condition
sensor.enable_interrupt(BMA400_GEN1_INT_EN, True)

int1 = Pin(INT1_PIN, Pin.IN)
int1.irq(trigger=Pin.IRQ_RISING, handler=int1_callback)

print("Waiting for motion...")

while True:
    if mems_event:
        mems_event = False
        status = sensor.get_interrupt_status()
        if status is not None and (status & BMA400_ASSERTED_GEN1_INT):
            print("Motion detected!")
