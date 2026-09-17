# FILE: bma400-stepCounter.py
# AUTHOR: Soldered Electronics
# BRIEF: Uses the BMA400's built-in step counter and activity type detection
#        (running/walking/still)
# WORKS WITH: BMA400 Accelerometer breakout: www.solde.red/333419
# LAST UPDATED: 2026-09-17

from machine import I2C, Pin
from bma400 import (
    BMA400,
    BMA400_OK,
    BMA400_INT_CHANNEL_1,
    BMA400_INT_PUSH_PULL_ACTIVE_1,
    BMA400_STEP_COUNTER_INT_EN,
    BMA400_ASSERTED_STEP_INT,
    BMA400_RUN_ACT,
    BMA400_WALK_ACT,
    BMA400_STILL_ACT,
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

# Configure the step counter feature. Step detection and counting is handled
# entirely by the sensor; the count is a 24-bit value that can be read at any
# time
sensor.set_step_counter_interrupt({"int_chan": BMA400_INT_CHANNEL_1})

# Configure INT1 pin to push/pull mode, active high
sensor.set_interrupt_pin_mode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1)

# Enable the step counter interrupt condition. This must be enabled for step
# counting to work at all, even without using the interrupt pin
sensor.enable_interrupt(BMA400_STEP_COUNTER_INT_EN, True)

int1 = Pin(INT1_PIN, Pin.IN)
int1.irq(trigger=Pin.IRQ_RISING, handler=int1_callback)

print("Waiting for steps...")

activity_names = {
    BMA400_RUN_ACT: "Running",
    BMA400_WALK_ACT: "Walking",
    BMA400_STILL_ACT: "Standing still",
}

while True:
    if mems_event:
        mems_event = False
        status = sensor.get_interrupt_status()
        if status is not None and (status & BMA400_ASSERTED_STEP_INT):
            count, activity = sensor.get_step_count()
            if count is not None:
                print(
                    "Step detected! Step count: {}, activity: {}".format(
                        count, activity_names.get(activity, "Unknown")
                    )
                )
