# FILE: bma400.py
# AUTHOR: Soldered Electronics
# BRIEF: MicroPython driver for the Bosch BMA400 ultra-low-power triaxial
#        accelerometer, ported from the Soldered BMA400 Arduino library
#        (which wraps SparkFun's port of the Bosch BMA400 Sensor API,
#        BSD-3-Clause) and cross-checked against bma400.c/bma400_defs.h
# LAST UPDATED: 2026-09-17

from machine import I2C, Pin
from os import uname
import time

# I2C addresses
BMA400_I2C_ADDRESS_SDO_LOW = 0x14
BMA400_I2C_ADDRESS_SDO_HIGH = 0x15

# Chip identifier and command bytes
BMA400_CHIP_ID = 0x90
BMA400_SOFT_RESET_CMD = 0xB6
BMA400_FIFO_FLUSH_CMD = 0xB0

# Return codes of the driver
BMA400_OK = 0
BMA400_E_NULL_PTR = -1
BMA400_E_COM_FAIL = -2
BMA400_E_DEV_NOT_FOUND = -3
BMA400_E_INVALID_CONFIG = -4
BMA400_W_SELF_TEST_FAIL = 1

# Aggregated results of check_status()
BMA400_ERROR = -1
BMA400_WARNING = 1

# Power mode configurations
BMA400_MODE_NORMAL = 0x02
BMA400_MODE_SLEEP = 0x00
BMA400_MODE_LOW_POWER = 0x01

# Enable / disable
BMA400_DISABLE = 0
BMA400_ENABLE = 1

# Data/sensortime selection for get_sensor_data()
BMA400_DATA_ONLY = 0x00
BMA400_DATA_SENSOR_TIME = 0x01

# Output data rate configurations
BMA400_ODR_12_5HZ = 0x05
BMA400_ODR_25HZ = 0x06
BMA400_ODR_50HZ = 0x07
BMA400_ODR_100HZ = 0x08
BMA400_ODR_200HZ = 0x09
BMA400_ODR_400HZ = 0x0A
BMA400_ODR_800HZ = 0x0B

# Accelerometer measurement range
BMA400_RANGE_2G = 0x00
BMA400_RANGE_4G = 0x01
BMA400_RANGE_8G = 0x02
BMA400_RANGE_16G = 0x03

# Axis selection, used by wakeup / orientation / generic / activity-change interrupts
BMA400_AXIS_X_EN = 0x01
BMA400_AXIS_Y_EN = 0x02
BMA400_AXIS_Z_EN = 0x04
BMA400_AXIS_XYZ_EN = 0x07

# Accel filter (data_src) selection
BMA400_DATA_SRC_ACCEL_FILT_1 = 0x00
BMA400_DATA_SRC_ACCEL_FILT_2 = 0x01
BMA400_DATA_SRC_ACCEL_FILT_LP = 0x02

# Accel oversampling (osr, osr_lp) settings
BMA400_ACCEL_OSR_SETTING_0 = 0x00
BMA400_ACCEL_OSR_SETTING_1 = 0x01
BMA400_ACCEL_OSR_SETTING_2 = 0x02
BMA400_ACCEL_OSR_SETTING_3 = 0x03

# Accel filt1_bw settings: FILT1_BW_0 = 0.48*ODR, FILT1_BW_1 = 0.24*ODR
BMA400_ACCEL_FILT1_BW_0 = 0x00
BMA400_ACCEL_FILT1_BW_1 = 0x01

# Interrupt channel selection
BMA400_UNMAP_INT_PIN = 0
BMA400_INT_CHANNEL_1 = 1
BMA400_INT_CHANNEL_2 = 2
BMA400_MAP_BOTH_INT_PINS = 3

# Interrupt pin hardware configuration
BMA400_INT_PUSH_PULL_ACTIVE_0 = 0x00
BMA400_INT_PUSH_PULL_ACTIVE_1 = 0x01
BMA400_INT_OPEN_DRIVE_ACTIVE_0 = 0x02
BMA400_INT_OPEN_DRIVE_ACTIVE_1 = 0x03

# Interrupt selection for enable_interrupt(), matches enum bma400_int_type
BMA400_DRDY_INT_EN = 0
BMA400_FIFO_WM_INT_EN = 1
BMA400_FIFO_FULL_INT_EN = 2
BMA400_GEN2_INT_EN = 3
BMA400_GEN1_INT_EN = 4
BMA400_ORIENT_CHANGE_INT_EN = 5
BMA400_LATCH_INT_EN = 6
BMA400_ACTIVITY_CHANGE_INT_EN = 7
BMA400_DOUBLE_TAP_INT_EN = 8
BMA400_SINGLE_TAP_INT_EN = 9
BMA400_STEP_COUNTER_INT_EN = 10
BMA400_AUTO_WAKEUP_EN = 11

# Interrupt assertion status flags, as returned by get_interrupt_status()
BMA400_ASSERTED_WAKEUP_INT = 0x0001
BMA400_ASSERTED_ORIENT_CH = 0x0002
BMA400_ASSERTED_GEN1_INT = 0x0004
BMA400_ASSERTED_GEN2_INT = 0x0008
BMA400_ASSERTED_INT_OVERRUN = 0x0010
BMA400_ASSERTED_FIFO_FULL_INT = 0x0020
BMA400_ASSERTED_FIFO_WM_INT = 0x0040
BMA400_ASSERTED_DRDY_INT = 0x0080
BMA400_ASSERTED_STEP_INT = 0x0300
BMA400_ASSERTED_S_TAP_INT = 0x0400
BMA400_ASSERTED_D_TAP_INT = 0x0800
BMA400_ASSERTED_ACT_CH_X = 0x2000
BMA400_ASSERTED_ACT_CH_Y = 0x4000
BMA400_ASSERTED_ACT_CH_Z = 0x8000

# Generic interrupt criterion_sel
BMA400_ACTIVITY_INT = 0x01
BMA400_INACTIVITY_INT = 0x00

# Generic/orientation/activity-change interrupt axes evaluation logic
BMA400_ALL_AXES_INT = 0x01
BMA400_ANY_AXES_INT = 0x00

# Generic interrupt hysteresis
BMA400_HYST_0_MG = 0x00
BMA400_HYST_24_MG = 0x01
BMA400_HYST_48_MG = 0x02
BMA400_HYST_96_MG = 0x03

# Reference update mode, used by generic/wakeup interrupts
BMA400_UPDATE_MANUAL = 0x00
BMA400_UPDATE_ONE_TIME = 0x01
BMA400_UPDATE_EVERY_TIME = 0x02
BMA400_UPDATE_LP_EVERY_TIME = 0x03

# Reference update mode, orientation interrupt only
BMA400_ORIENT_REFU_ACC_FILT_2 = 0x01
BMA400_ORIENT_REFU_ACC_FILT_LP = 0x02

# Wakeup interrupt sample count
BMA400_SAMPLE_COUNT_1 = 0x00
BMA400_SAMPLE_COUNT_2 = 0x01
BMA400_SAMPLE_COUNT_3 = 0x02
BMA400_SAMPLE_COUNT_4 = 0x03
BMA400_SAMPLE_COUNT_5 = 0x04
BMA400_SAMPLE_COUNT_6 = 0x05
BMA400_SAMPLE_COUNT_7 = 0x06
BMA400_SAMPLE_COUNT_8 = 0x07

# Auto low power trigger
BMA400_AUTO_LP_TIMEOUT_DISABLE = 0x00
BMA400_AUTO_LP_DRDY_TRIGGER = 0x01
BMA400_AUTO_LP_GEN1_TRIGGER = 0x02
BMA400_AUTO_LP_TIMEOUT_EN = 0x04
BMA400_AUTO_LP_TIME_RESET_EN = 0x08

# Tap interrupt axes selection
BMA400_TAP_X_AXIS_EN = 0x02
BMA400_TAP_Y_AXIS_EN = 0x01
BMA400_TAP_Z_AXIS_EN = 0x00

# Tap tics_th (max samples between upper/lower peak of a tap)
BMA400_TICS_TH_6_DATA_SAMPLES = 0x00
BMA400_TICS_TH_9_DATA_SAMPLES = 0x01
BMA400_TICS_TH_12_DATA_SAMPLES = 0x02
BMA400_TICS_TH_18_DATA_SAMPLES = 0x03

# Tap sensitivity, 0 = highest sensitivity, 7 = lowest
BMA400_TAP_SENSITIVITY_0 = 0x00
BMA400_TAP_SENSITIVITY_1 = 0x01
BMA400_TAP_SENSITIVITY_2 = 0x02
BMA400_TAP_SENSITIVITY_3 = 0x03
BMA400_TAP_SENSITIVITY_4 = 0x04
BMA400_TAP_SENSITIVITY_5 = 0x05
BMA400_TAP_SENSITIVITY_6 = 0x06
BMA400_TAP_SENSITIVITY_7 = 0x07

# Tap quiet time (samples of quiet time before/after single or double tap)
BMA400_QUIET_60_DATA_SAMPLES = 0x00
BMA400_QUIET_80_DATA_SAMPLES = 0x01
BMA400_QUIET_100_DATA_SAMPLES = 0x02
BMA400_QUIET_120_DATA_SAMPLES = 0x03

# Tap quiet_dt (minimum samples between the two taps of a double tap)
BMA400_QUIET_DT_4_DATA_SAMPLES = 0x00
BMA400_QUIET_DT_8_DATA_SAMPLES = 0x01
BMA400_QUIET_DT_12_DATA_SAMPLES = 0x02
BMA400_QUIET_DT_16_DATA_SAMPLES = 0x03

# Activity-change data source
BMA400_DATA_SRC_ACC_FILT1 = 0x00
BMA400_DATA_SRC_ACC_FILT2 = 0x01

# Activity-change sample count
BMA400_ACT_CH_SAMPLE_CNT_32 = 0x00
BMA400_ACT_CH_SAMPLE_CNT_64 = 0x01
BMA400_ACT_CH_SAMPLE_CNT_128 = 0x02
BMA400_ACT_CH_SAMPLE_CNT_256 = 0x03
BMA400_ACT_CH_SAMPLE_CNT_512 = 0x04

# Step status field - activity classification, from get_step_count()
BMA400_STILL_ACT = 0x00
BMA400_WALK_ACT = 0x01
BMA400_RUN_ACT = 0x02

# FIFO config flags (bma400_fifo_conf.conf_regs)
BMA400_FIFO_AUTO_FLUSH = 0x01
BMA400_FIFO_STOP_ON_FULL = 0x02
BMA400_FIFO_TIME_EN = 0x04
BMA400_FIFO_DATA_SRC = 0x08
BMA400_FIFO_8_BIT_EN = 0x10
BMA400_FIFO_X_EN = 0x20
BMA400_FIFO_Y_EN = 0x40
BMA400_FIFO_Z_EN = 0x80

# FIFO frame header values, as found in the raw FIFO byte stream
_BMA400_FIFO_X_ENABLE = 0x82
_BMA400_FIFO_Y_ENABLE = 0x84
_BMA400_FIFO_Z_ENABLE = 0x88
_BMA400_FIFO_XY_ENABLE = 0x86
_BMA400_FIFO_YZ_ENABLE = 0x8C
_BMA400_FIFO_XZ_ENABLE = 0x8A
_BMA400_FIFO_XYZ_ENABLE = 0x8E
_BMA400_FIFO_SENSOR_TIME = 0xA0
_BMA400_FIFO_EMPTY_FRAME = 0x80
_BMA400_FIFO_CONTROL_FRAME = 0x48
_BMA400_AWIDTH_MASK = 0xEF
_BMA400_FIFO_DATA_EN_MASK = 0x0E
_BMA400_FIFO_BYTES_OVERREAD = 25

# Self test configuration and pass/fail thresholds (raw LSB units, 4G range)
_BMA400_SELF_TEST_DISABLE = 0x00
_BMA400_SELF_TEST_ENABLE_POSITIVE = 0x07
_BMA400_SELF_TEST_ENABLE_NEGATIVE = 0x0F
_BMA400_ST_ACC_X_AXIS_SIGNAL_DIFF = 768
_BMA400_ST_ACC_Y_AXIS_SIGNAL_DIFF = 614
_BMA400_ST_ACC_Z_AXIS_SIGNAL_DIFF = 128

# Register map
BMA400_REG_CHIP_ID = 0x00
BMA400_REG_STATUS = 0x03
BMA400_REG_ACCEL_DATA = 0x04
BMA400_REG_INT_STAT0 = 0x0E
BMA400_REG_TEMP_DATA = 0x11
BMA400_REG_FIFO_LENGTH = 0x12
BMA400_REG_FIFO_DATA = 0x14
BMA400_REG_STEP_CNT_0 = 0x15
BMA400_REG_ACCEL_CONFIG_0 = 0x19
BMA400_REG_ACCEL_CONFIG_1 = 0x1A
BMA400_REG_ACCEL_CONFIG_2 = 0x1B
BMA400_REG_INT_CONF_0 = 0x1F
BMA400_REG_INT_CONF_1 = 0x20
BMA400_REG_INT_MAP = 0x21
BMA400_REG_INT_12_IO_CTRL = 0x24
BMA400_REG_FIFO_CONFIG_0 = 0x26
BMA400_REG_FIFO_READ_EN = 0x29
BMA400_REG_AUTO_LOW_POW_0 = 0x2A
BMA400_REG_AUTO_LOW_POW_1 = 0x2B
BMA400_REG_AUTOWAKEUP_0 = 0x2C
BMA400_REG_AUTOWAKEUP_1 = 0x2D
BMA400_REG_WAKEUP_INT_CONF_0 = 0x2F
BMA400_REG_ORIENTCH_INT_CONFIG = 0x35
BMA400_REG_GEN1_INT_CONFIG = 0x3F
BMA400_REG_GEN2_INT_CONFIG = 0x4A
BMA400_REG_ACT_CH_CONFIG_0 = 0x55
BMA400_REG_TAP_CONFIG = 0x57
BMA400_REG_SELF_TEST = 0x7D
BMA400_REG_COMMAND = 0x7E

# Bit masks / positions used to pack and unpack the registers above. Named to
# match the Bosch driver's macros (bma400_defs.h) so they can be cross-checked
# against it directly.
_BMA400_POWER_MODE_MSK = 0x03
_BMA400_POWER_MODE_STATUS_MSK = 0x06
_BMA400_POWER_MODE_STATUS_POS = 1
_BMA400_ACCEL_ODR_MSK = 0x0F
_BMA400_ACCEL_RANGE_MSK = 0xC0
_BMA400_ACCEL_RANGE_POS = 6
_BMA400_DATA_FILTER_MSK = 0x0C
_BMA400_DATA_FILTER_POS = 2
_BMA400_OSR_MSK = 0x30
_BMA400_OSR_POS = 4
_BMA400_OSR_LP_MSK = 0x60
_BMA400_OSR_LP_POS = 5
_BMA400_FILT_1_BW_MSK = 0x80
_BMA400_FILT_1_BW_POS = 7
_BMA400_INT_PIN1_CONF_MSK = 0x06
_BMA400_INT_PIN1_CONF_POS = 1
_BMA400_INT_PIN2_CONF_MSK = 0x60
_BMA400_INT_PIN2_CONF_POS = 5
_BMA400_INT_STATUS_MSK = 0xE0
_BMA400_INT_STATUS_POS = 5
_BMA400_EN_DRDY_MSK = 0x80
_BMA400_EN_DRDY_POS = 7
_BMA400_EN_FIFO_WM_MSK = 0x40
_BMA400_EN_FIFO_WM_POS = 6
_BMA400_EN_FIFO_FULL_MSK = 0x20
_BMA400_EN_FIFO_FULL_POS = 5
_BMA400_EN_INT_OVERRUN_MSK = 0x10
_BMA400_EN_INT_OVERRUN_POS = 4
_BMA400_EN_GEN2_MSK = 0x08
_BMA400_EN_GEN2_POS = 3
_BMA400_EN_GEN1_MSK = 0x04
_BMA400_EN_GEN1_POS = 2
_BMA400_EN_ORIENT_CH_MSK = 0x02
_BMA400_EN_ORIENT_CH_POS = 1
_BMA400_EN_LATCH_MSK = 0x80
_BMA400_EN_LATCH_POS = 7
_BMA400_EN_ACTCH_MSK = 0x10
_BMA400_EN_ACTCH_POS = 4
_BMA400_EN_D_TAP_MSK = 0x08
_BMA400_EN_D_TAP_POS = 3
_BMA400_EN_S_TAP_MSK = 0x04
_BMA400_EN_S_TAP_POS = 2
_BMA400_EN_STEP_INT_MSK = 0x01
_BMA400_EN_WAKEUP_INT_MSK = 0x01
_BMA400_WAKEUP_INTERRUPT_MSK = 0x02
_BMA400_WAKEUP_INTERRUPT_POS = 1
_BMA400_WAKEUP_TIMEOUT_MSK = 0x04
_BMA400_WAKEUP_TIMEOUT_POS = 2
_BMA400_WAKEUP_TIMEOUT_THRES_MSK = 0xF0
_BMA400_WAKEUP_TIMEOUT_THRES_POS = 4
_BMA400_WKUP_REF_UPDATE_MSK = 0x03
_BMA400_SAMPLE_COUNT_MSK = 0x1C
_BMA400_SAMPLE_COUNT_POS = 2
_BMA400_WAKEUP_EN_AXES_MSK = 0xE0
_BMA400_WAKEUP_EN_AXES_POS = 5
_BMA400_AUTO_LOW_POW_MSK = 0x0F
_BMA400_AUTO_LP_THRES_MSK = 0x0FF0
_BMA400_AUTO_LP_THRES_POS = 4
_BMA400_AUTO_LP_THRES_LSB_MSK = 0x000F
_BMA400_AUTO_LP_TIMEOUT_LSB_MSK = 0xF0
_BMA400_AUTO_LP_TIMEOUT_LSB_POS = 4
_BMA400_TAP_AXES_EN_MSK = 0x18
_BMA400_TAP_AXES_EN_POS = 3
_BMA400_TAP_SENSITIVITY_MSK = 0x07
_BMA400_TAP_QUIET_DT_MSK = 0x30
_BMA400_TAP_QUIET_DT_POS = 4
_BMA400_TAP_QUIET_MSK = 0x0C
_BMA400_TAP_QUIET_POS = 2
_BMA400_TAP_TICS_TH_MSK = 0x03
_BMA400_TAP_MAP_INT1_MSK = 0x04
_BMA400_TAP_MAP_INT1_POS = 2
_BMA400_TAP_MAP_INT2_MSK = 0x40
_BMA400_TAP_MAP_INT2_POS = 6
_BMA400_ACT_CH_AXES_EN_MSK = 0xE0
_BMA400_ACT_CH_AXES_EN_POS = 5
_BMA400_ACT_CH_DATA_SRC_MSK = 0x10
_BMA400_ACT_CH_DATA_SRC_POS = 4
_BMA400_ACT_CH_NPTS_MSK = 0x0F
_BMA400_ACTCH_MAP_INT1_MSK = 0x08
_BMA400_ACTCH_MAP_INT1_POS = 3
_BMA400_ACTCH_MAP_INT2_MSK = 0x80
_BMA400_ACTCH_MAP_INT2_POS = 7
_BMA400_STEP_MAP_INT2_MSK = 0x10
_BMA400_STEP_MAP_INT2_POS = 4
_BMA400_INT_AXES_EN_MSK = 0xE0
_BMA400_INT_AXES_EN_POS = 5
_BMA400_INT_DATA_SRC_MSK = 0x10
_BMA400_INT_DATA_SRC_POS = 4
_BMA400_INT_REFU_MSK = 0x0C
_BMA400_INT_REFU_POS = 2
_BMA400_INT_HYST_MSK = 0x03
_BMA400_GEN_INT_COMB_MSK = 0x01
_BMA400_GEN_INT_CRITERION_MSK = 0x02
_BMA400_GEN_INT_CRITERION_POS = 1
_BMA400_FIFO_BYTES_CNT_MSK = 0x07
_BMA400_FIFO_TIME_EN_MSK = 0x04
_BMA400_FIFO_TIME_EN_POS = 2
_BMA400_FIFO_AXES_EN_MSK = 0xE0
_BMA400_FIFO_AXES_EN_POS = 5
_BMA400_FIFO_8_BIT_EN_MSK = 0x10
_BMA400_FIFO_8_BIT_EN_POS = 4

# Text descriptions of the status codes, used by status_string()
_STATUS_STRINGS = {
    BMA400_OK: "",
    BMA400_E_NULL_PTR: "Null pointer",
    BMA400_E_COM_FAIL: "Communication failure",
    BMA400_E_DEV_NOT_FOUND: "Sensor not found",
    BMA400_E_INVALID_CONFIG: "Invalid configuration",
    BMA400_W_SELF_TEST_FAIL: "Self test failed",
}


def _s12(value):
    """Interpret a 12 bit unsigned value as a signed integer."""
    return value - 4096 if value > 2047 else value


class BMA400Data:
    """One accelerometer reading, converted to g's."""

    def __init__(self):
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.sensor_time_ms = 0

    def __repr__(self):
        return "BMA400Data(x={:.3f}, y={:.3f}, z={:.3f}, sensor_time_ms={})".format(
            self.accel_x, self.accel_y, self.accel_z, self.sensor_time_ms
        )


class BMA400:
    """
    MicroPython driver for the Soldered BMA400 breakout board, I2C only.

    The sensor is initialized by the constructor, which raises an exception
    when it cannot be reached. Every other method stores its result in the
    status attribute instead of raising, the same way the Arduino library
    does, so check_status() and status_string() report what went wrong.
    """

    def __init__(self, i2c=None, address=BMA400_I2C_ADDRESS_SDO_LOW):
        """
        Initialize the BMA400.

        :param i2c: Initialized I2C object, auto-detected on known boards
        :param address: I2C address, BMA400_I2C_ADDRESS_SDO_LOW by default
        """
        if i2c is not None:
            self.i2c = i2c
        else:
            if uname().sysname in (
                "esp32",
                "esp8266",
                "Soldered Dasduino CONNECTPLUS",
            ):
                self.i2c = I2C(0, scl=Pin(22), sda=Pin(21))
            else:
                raise Exception(
                    "Board not recognized, enter I2C pins manually"
                )

        self.address = address
        self.status = BMA400_OK
        self.intf_rslt = BMA400_OK
        self.chip_id = 0
        self.data = BMA400Data()

        self._init_sensor()

    # ------------------------------------------------------------------
    # Bus access
    # ------------------------------------------------------------------

    def _get_regs(self, reg_addr, length):
        """Read a block of registers, raises OSError on a bus failure."""
        data = self.i2c.readfrom_mem(self.address, reg_addr, length)
        self.intf_rslt = BMA400_OK
        return data

    def _set_regs(self, reg_addr, reg_data):
        """
        Write a block of sequential registers.

        Unlike reads, the BMA400 does not support burst writes across
        multiple registers -- the Bosch reference driver explicitly writes
        each byte of a multi-byte block as its own single-byte transaction,
        incrementing the register address each time (see bma400_set_regs()
        in bma400.c: "Burst write is not allowed thus we split burst case
        write into single byte writes"). A single multi-byte writeto_mem()
        call here only lands the first byte; every register past it is left
        unwritten.
        """
        for offset, byte in enumerate(reg_data):
            self.i2c.writeto_mem(self.address, reg_addr + offset, bytes([byte]))
        self.intf_rslt = BMA400_OK

    def read_reg(self, reg_addr, length=1):
        """
        Read one or more registers.

        :param reg_addr: Address of the first register
        :param length: Number of bytes to read
        :return: An integer for a single byte, bytes otherwise, None on error
        """
        try:
            data = self._get_regs(reg_addr, length)
            self.status = BMA400_OK
            return data[0] if length == 1 else data
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL
            return None

    def read_regs(self, reg_addr, length):
        """Alias of read_reg(), kept for parity with the Arduino/ESP-IDF ports."""
        return self.read_reg(reg_addr, length)

    def write_reg(self, reg_addr, reg_data):
        """
        Write one or more registers.

        :param reg_addr: Address of the first register
        :param reg_data: Single byte, or a list/bytes of sequential data
        """
        if isinstance(reg_data, int):
            reg_data = [reg_data]

        try:
            self._set_regs(reg_addr, reg_data)
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def write_regs(self, reg_addr, reg_data):
        """Alias of write_reg(), kept for parity with the Arduino/ESP-IDF ports."""
        self.write_reg(reg_addr, reg_data)

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _soft_reset_raw(self):
        self._set_regs(BMA400_REG_COMMAND, [BMA400_SOFT_RESET_CMD])
        time.sleep_us(5000)

    def _init_sensor(self):
        """Reset the sensor, check its chip ID, and set normal power mode."""
        try:
            self._soft_reset_raw()
            self.chip_id = self._get_regs(BMA400_REG_CHIP_ID, 1)[0]
            if self.chip_id != BMA400_CHIP_ID:
                self.status = BMA400_E_DEV_NOT_FOUND
                raise Exception(
                    "BMA400 not found, chip ID 0x{:02x} was read instead of "
                    "0x{:02x}".format(self.chip_id, BMA400_CHIP_ID)
                )

            self._set_mode_raw(BMA400_MODE_NORMAL)
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL
            raise Exception(
                "BMA400 not responding on address 0x{:02x}".format(
                    self.address
                )
            )

    def soft_reset(self):
        """Soft reset the sensor, the configuration is lost."""
        try:
            self._soft_reset_raw()
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    # ------------------------------------------------------------------
    # Power mode
    # ------------------------------------------------------------------

    def _set_mode_raw(self, mode):
        reg_data = self._get_regs(BMA400_REG_ACCEL_CONFIG_0, 1)[0]
        reg_data = (reg_data & ~_BMA400_POWER_MODE_MSK & 0xFF) | (
            mode & _BMA400_POWER_MODE_MSK
        )
        self._set_regs(BMA400_REG_ACCEL_CONFIG_0, [reg_data])

        # A delay of 1/ODR is required to switch power modes. Low power mode
        # has a 25Hz frequency and hence needs 40ms; the other modes settle
        # within 10ms.
        time.sleep_us(40000 if mode == BMA400_MODE_LOW_POWER else 10000)

    def set_mode(self, mode):
        """
        Set the power mode.

        :param mode: BMA400_MODE_NORMAL, BMA400_MODE_SLEEP or BMA400_MODE_LOW_POWER
        """
        try:
            self._set_mode_raw(mode)
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def get_mode(self):
        """
        Get the power mode the sensor is actually running in.

        :return: BMA400_MODE_NORMAL, BMA400_MODE_SLEEP or BMA400_MODE_LOW_POWER,
                 None on error
        """
        try:
            reg_data = self._get_regs(BMA400_REG_STATUS, 1)[0]
            self.status = BMA400_OK
            return (
                reg_data & _BMA400_POWER_MODE_STATUS_MSK
            ) >> _BMA400_POWER_MODE_STATUS_POS
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL
            return None

    # ------------------------------------------------------------------
    # Accelerometer configuration
    #
    # The sensor groups all accelerometer settings into one 3 byte block
    # (ACCEL_CONFIG_0/1/2) that is read-modify-written as a whole, so every
    # setter below first fetches the current configuration to avoid
    # clobbering unrelated fields.
    # ------------------------------------------------------------------

    def _get_accel_conf_raw(self):
        data = self._get_regs(BMA400_REG_ACCEL_CONFIG_0, 3)
        return {
            "filt1_bw": (data[0] & _BMA400_FILT_1_BW_MSK) >> _BMA400_FILT_1_BW_POS,
            "osr_lp": (data[0] & _BMA400_OSR_LP_MSK) >> _BMA400_OSR_LP_POS,
            "range": (data[1] & _BMA400_ACCEL_RANGE_MSK) >> _BMA400_ACCEL_RANGE_POS,
            "osr": (data[1] & _BMA400_OSR_MSK) >> _BMA400_OSR_POS,
            "odr": data[1] & _BMA400_ACCEL_ODR_MSK,
            "data_src": (data[2] & _BMA400_DATA_FILTER_MSK) >> _BMA400_DATA_FILTER_POS,
        }

    def _set_accel_conf_raw(self, conf):
        data = bytearray(self._get_regs(BMA400_REG_ACCEL_CONFIG_0, 3))
        data[0] = (data[0] & ~_BMA400_FILT_1_BW_MSK & 0xFF) | (
            (conf["filt1_bw"] << _BMA400_FILT_1_BW_POS) & _BMA400_FILT_1_BW_MSK
        )
        data[0] = (data[0] & ~_BMA400_OSR_LP_MSK & 0xFF) | (
            (conf["osr_lp"] << _BMA400_OSR_LP_POS) & _BMA400_OSR_LP_MSK
        )
        data[1] = (data[1] & ~_BMA400_ACCEL_RANGE_MSK & 0xFF) | (
            (conf["range"] << _BMA400_ACCEL_RANGE_POS) & _BMA400_ACCEL_RANGE_MSK
        )
        data[1] = (data[1] & ~_BMA400_OSR_MSK & 0xFF) | (
            (conf["osr"] << _BMA400_OSR_POS) & _BMA400_OSR_MSK
        )
        data[1] = (data[1] & ~_BMA400_ACCEL_ODR_MSK & 0xFF) | (
            conf["odr"] & _BMA400_ACCEL_ODR_MSK
        )
        data[2] = (data[2] & ~_BMA400_DATA_FILTER_MSK & 0xFF) | (
            (conf["data_src"] << _BMA400_DATA_FILTER_POS) & _BMA400_DATA_FILTER_MSK
        )
        self._set_regs(BMA400_REG_ACCEL_CONFIG_0, data)

    def _update_accel_conf(self, key, value):
        try:
            conf = self._get_accel_conf_raw()
            conf[key] = value
            self._set_accel_conf_raw(conf)
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def _read_accel_conf(self, key):
        try:
            conf = self._get_accel_conf_raw()
            self.status = BMA400_OK
            return conf[key]
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL
            return None

    def set_range(self, range_):
        """
        Set the measurement range.

        :param range_: BMA400_RANGE_2G, BMA400_RANGE_4G (default), BMA400_RANGE_8G
                        or BMA400_RANGE_16G
        """
        self._update_accel_conf("range", range_)

    def get_range(self):
        """:return: BMA400_RANGE_2G to BMA400_RANGE_16G, None on error"""
        return self._read_accel_conf("range")

    def set_odr(self, odr):
        """
        Set the output data rate.

        :param odr: BMA400_ODR_12_5HZ up to BMA400_ODR_800HZ (default BMA400_ODR_200HZ)
        """
        self._update_accel_conf("odr", odr)

    def get_odr(self):
        """:return: BMA400_ODR_12_5HZ up to BMA400_ODR_800HZ, None on error"""
        return self._read_accel_conf("odr")

    def set_osr(self, osr):
        """
        Set the oversampling rate.

        :param osr: BMA400_ACCEL_OSR_SETTING_0 (default) up to _SETTING_3
        """
        self._update_accel_conf("osr", osr)

    def get_osr(self):
        """:return: BMA400_ACCEL_OSR_SETTING_0 up to _SETTING_3, None on error"""
        return self._read_accel_conf("osr")

    def set_osr_lp(self, osr_lp):
        """
        Set the low power mode oversampling rate.

        :param osr_lp: BMA400_ACCEL_OSR_SETTING_0 (default) up to _SETTING_3
        """
        self._update_accel_conf("osr_lp", osr_lp)

    def get_osr_lp(self):
        """:return: BMA400_ACCEL_OSR_SETTING_0 up to _SETTING_3, None on error"""
        return self._read_accel_conf("osr_lp")

    def set_data_source(self, source):
        """
        Set the data source used for accelerometer readings.

        :param source: BMA400_DATA_SRC_ACCEL_FILT_1 (default), _FILT_2 or _FILT_LP
        """
        self._update_accel_conf("data_src", source)

    def get_data_source(self):
        """:return: BMA400_DATA_SRC_ACCEL_FILT_1/2/LP, None on error"""
        return self._read_accel_conf("data_src")

    def set_filter1_bandwidth(self, bw):
        """
        Set filter 1 bandwidth.

        :param bw: BMA400_ACCEL_FILT1_BW_0 (default, 0.48*ODR) or _BW_1 (0.24*ODR)
        """
        self._update_accel_conf("filt1_bw", bw)

    def get_filter1_bandwidth(self):
        """:return: BMA400_ACCEL_FILT1_BW_0 or _BW_1, None on error"""
        return self._read_accel_conf("filt1_bw")

    def set_drdy_interrupt_channel(self, channel):
        """
        Set the interrupt pin used for the data-ready interrupt condition.

        :param channel: BMA400_UNMAP_INT_PIN, BMA400_INT_CHANNEL_1,
                         BMA400_INT_CHANNEL_2 or BMA400_MAP_BOTH_INT_PINS
        """
        try:
            self._map_int_pin("drdy", channel)
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    # ------------------------------------------------------------------
    # Step counter and self test
    # ------------------------------------------------------------------

    def get_step_count(self):
        """
        Get the step count and detected activity type.

        :return: Tuple of (count, activity_type), where activity_type is
                 BMA400_STILL_ACT, BMA400_WALK_ACT or BMA400_RUN_ACT.
                 (None, None) on error
        """
        try:
            data = self._get_regs(BMA400_REG_STEP_CNT_0, 4)
            self.status = BMA400_OK
            count = data[0] | (data[1] << 8) | (data[2] << 16)
            return count, data[3]
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL
            return None, None

    def self_test(self):
        """
        Run the built-in self test.

        Physically pushes the sensing element to determine whether it is
        behaving correctly. Leaves the sensor in normal power mode
        afterwards (a successful test soft-resets the sensor internally).

        :return: BMA400_OK if the self test passed, BMA400_W_SELF_TEST_FAIL
                 if it completed but the readings were out of range, or a
                 negative error code on a bus failure
        """
        try:
            self._enable_self_test()
            accel_pos = self._positive_excited_accel()
            accel_neg = self._negative_excited_accel()
            self.status = self._validate_self_test(accel_pos, accel_neg)
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL
            return self.status

        # A successful (or failed, but bus-healthy) self test always ends
        # with a soft reset, matching bma400_perform_self_test(); the sensor
        # has to be reconfigured afterwards.
        self_test_rslt = self.status
        try:
            self._soft_reset_raw()
            self._set_mode_raw(BMA400_MODE_NORMAL)
            self.status = self_test_rslt
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

        return self.status

    def _enable_self_test(self):
        """Configure the accelerometer the way the self test procedure expects."""
        conf = self._get_accel_conf_raw()
        conf["odr"] = BMA400_ODR_100HZ
        conf["range"] = BMA400_RANGE_4G
        conf["osr"] = BMA400_ACCEL_OSR_SETTING_3
        conf["data_src"] = BMA400_DATA_SRC_ACCEL_FILT_1
        self._set_accel_conf_raw(conf)
        time.sleep_us(7000)
        self._set_mode_raw(BMA400_MODE_NORMAL)

    def _positive_excited_accel(self):
        self._set_regs(BMA400_REG_SELF_TEST, [_BMA400_SELF_TEST_ENABLE_POSITIVE])
        time.sleep_us(50000)
        return self._get_accel_raw(BMA400_DATA_ONLY)

    def _negative_excited_accel(self):
        self._set_regs(BMA400_REG_SELF_TEST, [_BMA400_SELF_TEST_ENABLE_NEGATIVE])
        time.sleep_us(50000)
        accel = self._get_accel_raw(BMA400_DATA_ONLY)
        self._set_regs(BMA400_REG_SELF_TEST, [_BMA400_SELF_TEST_DISABLE])
        return accel

    @staticmethod
    def _validate_self_test(accel_pos, accel_neg):
        diff_x = accel_pos[0] - accel_neg[0]
        diff_y = accel_pos[1] - accel_neg[1]
        diff_z = accel_pos[2] - accel_neg[2]

        if (
            diff_x > _BMA400_ST_ACC_X_AXIS_SIGNAL_DIFF
            and diff_y > _BMA400_ST_ACC_Y_AXIS_SIGNAL_DIFF
            and diff_z > _BMA400_ST_ACC_Z_AXIS_SIGNAL_DIFF
        ):
            return BMA400_OK

        return BMA400_W_SELF_TEST_FAIL

    # ------------------------------------------------------------------
    # Reading out measurements
    # ------------------------------------------------------------------

    def _get_accel_raw(self, data_sel):
        """
        Read raw signed 12 bit accelerometer counts (and optionally sensor
        time), without converting to g's. Used directly by self_test(),
        which compares raw counts against fixed thresholds.

        :return: Tuple of (x, y, z, sensortime)
        """
        length = 9 if data_sel == BMA400_DATA_SENSOR_TIME else 6
        data = self._get_regs(BMA400_REG_ACCEL_DATA, length)

        x = _s12(data[0] | (data[1] << 8))
        y = _s12(data[2] | (data[3] << 8))
        z = _s12(data[4] | (data[5] << 8))

        sensortime = 0
        if data_sel == BMA400_DATA_SENSOR_TIME:
            sensortime = data[6] | (data[7] << 8) | (data[8] << 16)

        return x, y, z, sensortime

    def get_sensor_data(self, sensor_time=False):
        """
        Read a new accelerometer measurement into self.data.

        :param sensor_time: Whether to also read the sensor's internal time
                             counter into self.data.sensor_time_ms
        :return: BMA400_OK on success, an error code otherwise
        """
        try:
            data_sel = BMA400_DATA_SENSOR_TIME if sensor_time else BMA400_DATA_ONLY
            x, y, z, sensortime = self._get_accel_raw(data_sel)
            range_ = self._get_accel_conf_raw()["range"]

            # rangeSetting -> g-range: RANGE_2G=2, _4G=4, _8G=8, _16G=16.
            # Raw data are signed 12 bit integers, where the maximum raw
            # value corresponds to the max of the range setting.
            g_range = 2 << range_
            raw_to_g = g_range / 2048.0

            self.data.accel_x = x * raw_to_g
            self.data.accel_y = y * raw_to_g
            self.data.accel_z = z * raw_to_g
            # Sensor time register increments at 25.6kHz
            self.data.sensor_time_ms = sensortime * 1000 // 25600

            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

        return self.status

    def get_temperature(self):
        """
        Get the temperature of the sensor.

        :return: Temperature in degrees Celsius, None on error
        """
        try:
            reg_data = self._get_regs(BMA400_REG_TEMP_DATA, 1)[0]
            self.status = BMA400_OK
            raw = reg_data - 256 if reg_data > 127 else reg_data
            return (raw * 5 + 230) / 10.0
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL
            return None

    # ------------------------------------------------------------------
    # Interrupt pins and status
    # ------------------------------------------------------------------

    def set_interrupt_pin_mode(self, channel, mode):
        """
        Set an interrupt pin as push/pull or open drain, and active high or low.

        :param channel: BMA400_INT_CHANNEL_1 or BMA400_INT_CHANNEL_2
        :param mode: BMA400_INT_PUSH_PULL_ACTIVE_0, _ACTIVE_1 (default),
                     BMA400_INT_OPEN_DRIVE_ACTIVE_0 or _ACTIVE_1
        """
        try:
            reg_data = self._get_regs(BMA400_REG_INT_12_IO_CTRL, 1)[0]
            if channel == BMA400_INT_CHANNEL_1:
                reg_data = (reg_data & ~_BMA400_INT_PIN1_CONF_MSK & 0xFF) | (
                    (mode << _BMA400_INT_PIN1_CONF_POS) & _BMA400_INT_PIN1_CONF_MSK
                )
            elif channel == BMA400_INT_CHANNEL_2:
                reg_data = (reg_data & ~_BMA400_INT_PIN2_CONF_MSK & 0xFF) | (
                    (mode << _BMA400_INT_PIN2_CONF_POS) & _BMA400_INT_PIN2_CONF_MSK
                )
            self._set_regs(BMA400_REG_INT_12_IO_CTRL, [reg_data])
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def enable_interrupt(self, int_type, enable):
        """
        Enable or disable an interrupt condition.

        :param int_type: One of the BMA400_*_INT_EN constants
        :param enable: True to enable, False to disable
        """
        try:
            if int_type == BMA400_AUTO_WAKEUP_EN:
                reg_data = self._get_regs(BMA400_REG_AUTOWAKEUP_1, 1)[0]
                reg_data = (reg_data & ~_BMA400_WAKEUP_INTERRUPT_MSK & 0xFF) | (
                    ((1 if enable else 0) << _BMA400_WAKEUP_INTERRUPT_POS)
                    & _BMA400_WAKEUP_INTERRUPT_MSK
                )
                self._set_regs(BMA400_REG_AUTOWAKEUP_1, [reg_data])
                self.status = BMA400_OK
                return

            data = bytearray(self._get_regs(BMA400_REG_INT_CONF_0, 2))
            conf = 1 if enable else 0

            if int_type == BMA400_DRDY_INT_EN:
                data[0] = self._set_bits(data[0], _BMA400_EN_DRDY_MSK, _BMA400_EN_DRDY_POS, conf)
            elif int_type == BMA400_FIFO_WM_INT_EN:
                data[0] = self._set_bits(data[0], _BMA400_EN_FIFO_WM_MSK, _BMA400_EN_FIFO_WM_POS, conf)
            elif int_type == BMA400_FIFO_FULL_INT_EN:
                data[0] = self._set_bits(data[0], _BMA400_EN_FIFO_FULL_MSK, _BMA400_EN_FIFO_FULL_POS, conf)
            elif int_type == BMA400_GEN2_INT_EN:
                data[0] = self._set_bits(data[0], _BMA400_EN_GEN2_MSK, _BMA400_EN_GEN2_POS, conf)
            elif int_type == BMA400_GEN1_INT_EN:
                data[0] = self._set_bits(data[0], _BMA400_EN_GEN1_MSK, _BMA400_EN_GEN1_POS, conf)
            elif int_type == BMA400_ORIENT_CHANGE_INT_EN:
                data[0] = self._set_bits(data[0], _BMA400_EN_ORIENT_CH_MSK, _BMA400_EN_ORIENT_CH_POS, conf)
            elif int_type == BMA400_LATCH_INT_EN:
                data[1] = self._set_bits(data[1], _BMA400_EN_LATCH_MSK, _BMA400_EN_LATCH_POS, conf)
            elif int_type == BMA400_ACTIVITY_CHANGE_INT_EN:
                data[1] = self._set_bits(data[1], _BMA400_EN_ACTCH_MSK, _BMA400_EN_ACTCH_POS, conf)
            elif int_type == BMA400_DOUBLE_TAP_INT_EN:
                data[1] = self._set_bits(data[1], _BMA400_EN_D_TAP_MSK, _BMA400_EN_D_TAP_POS, conf)
            elif int_type == BMA400_SINGLE_TAP_INT_EN:
                data[1] = self._set_bits(data[1], _BMA400_EN_S_TAP_MSK, _BMA400_EN_S_TAP_POS, conf)
            elif int_type == BMA400_STEP_COUNTER_INT_EN:
                data[1] = (data[1] & ~_BMA400_EN_STEP_INT_MSK & 0xFF) | (
                    conf & _BMA400_EN_STEP_INT_MSK
                )
            else:
                self.status = BMA400_E_INVALID_CONFIG
                return

            self._set_regs(BMA400_REG_INT_CONF_0, data)
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    @staticmethod
    def _set_bits(reg_data, mask, pos, value):
        return (reg_data & ~mask & 0xFF) | ((value << pos) & mask)

    def get_interrupt_status(self):
        """
        Get the interrupt status flags.

        :return: Bitmask of BMA400_ASSERTED_* flags, None on error
        """
        try:
            data = self._get_regs(BMA400_REG_INT_STAT0, 3)
            self.status = BMA400_OK
            # The top 3 bits of byte 2 (activity-change X/Y/Z) are folded
            # into the top 3 bits of byte 1 before the two are concatenated.
            merged = (data[1] & ~_BMA400_INT_STATUS_MSK & 0xFF) | (
                (data[2] << _BMA400_INT_STATUS_POS) & _BMA400_INT_STATUS_MSK
            )
            return (merged << 8) | data[0]
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL
            return None

    def _map_int_pin(self, kind, channel):
        """
        Map (or unmap) one interrupt condition to INT1 and/or INT2.

        Direct port of bma400.c's map_int_pin(): reads the 3 byte INT_MAP
        block, sets or clears the relevant bit(s) for `kind`, and writes the
        block back. Each `kind` uses its own byte index and bit position,
        matching the sensor's register layout exactly.
        """
        data = bytearray(self._get_regs(BMA400_REG_INT_MAP, 3))
        ch1 = channel in (BMA400_INT_CHANNEL_1, BMA400_MAP_BOTH_INT_PINS)
        ch2 = channel in (BMA400_INT_CHANNEL_2, BMA400_MAP_BOTH_INT_PINS)

        # (byte index, mask, pos) for INT1 and INT2, for interrupts mapped in
        # bytes 0/1 with a shared bit position.
        shared_pos = {
            "drdy": (_BMA400_EN_DRDY_MSK, _BMA400_EN_DRDY_POS),
            "fifo_wm": (_BMA400_EN_FIFO_WM_MSK, _BMA400_EN_FIFO_WM_POS),
            "fifo_full": (_BMA400_EN_FIFO_FULL_MSK, _BMA400_EN_FIFO_FULL_POS),
            "int_overrun": (_BMA400_EN_INT_OVERRUN_MSK, _BMA400_EN_INT_OVERRUN_POS),
            "gen2": (_BMA400_EN_GEN2_MSK, _BMA400_EN_GEN2_POS),
            "gen1": (_BMA400_EN_GEN1_MSK, _BMA400_EN_GEN1_POS),
            "orient": (_BMA400_EN_ORIENT_CH_MSK, _BMA400_EN_ORIENT_CH_POS),
        }

        if kind in shared_pos:
            mask, pos = shared_pos[kind]
            data[0] = (data[0] & ~mask & 0xFF) | ((1 << pos) & mask if ch1 else 0)
            data[1] = (data[1] & ~mask & 0xFF) | ((1 << pos) & mask if ch2 else 0)
        elif kind == "wakeup":
            mask = _BMA400_EN_WAKEUP_INT_MSK
            data[0] = (data[0] & ~mask & 0xFF) | (mask if ch1 else 0)
            data[1] = (data[1] & ~mask & 0xFF) | (mask if ch2 else 0)
        elif kind == "act_ch":
            data[2] = (data[2] & ~_BMA400_ACTCH_MAP_INT1_MSK & 0xFF) | (
                _BMA400_ACTCH_MAP_INT1_MSK if ch1 else 0
            )
            data[2] = (data[2] & ~_BMA400_ACTCH_MAP_INT2_MSK & 0xFF) | (
                _BMA400_ACTCH_MAP_INT2_MSK if ch2 else 0
            )
        elif kind == "tap":
            data[2] = (data[2] & ~_BMA400_TAP_MAP_INT1_MSK & 0xFF) | (
                _BMA400_TAP_MAP_INT1_MSK if ch1 else 0
            )
            data[2] = (data[2] & ~_BMA400_TAP_MAP_INT2_MSK & 0xFF) | (
                _BMA400_TAP_MAP_INT2_MSK if ch2 else 0
            )
        elif kind == "step":
            data[2] = (data[2] & ~_BMA400_EN_STEP_INT_MSK & 0xFF) | (
                _BMA400_EN_STEP_INT_MSK if ch1 else 0
            )
            data[2] = (data[2] & ~_BMA400_STEP_MAP_INT2_MSK & 0xFF) | (
                _BMA400_STEP_MAP_INT2_MSK if ch2 else 0
            )

        self._set_regs(BMA400_REG_INT_MAP, data)

    # ------------------------------------------------------------------
    # Generic, orientation, tap, step counter and activity-change interrupts
    #
    # Each takes a plain dict matching the fields of the corresponding Bosch
    # struct (see bma400_defs.h). Unset keys default to 0.
    # ------------------------------------------------------------------

    def set_generic1_interrupt(self, config):
        """
        Set the generic interrupt 1 configuration.

        :param config: dict with keys gen_int_thres, gen_int_dur, axes_sel,
                        data_src, criterion_sel, evaluate_axes, ref_update,
                        hysteresis, int_thres_ref_x/y/z, int_chan
        """
        self._set_generic_interrupt(BMA400_REG_GEN1_INT_CONFIG, "gen1", config)

    def set_generic2_interrupt(self, config):
        """Same as set_generic1_interrupt(), for generic interrupt 2."""
        self._set_generic_interrupt(BMA400_REG_GEN2_INT_CONFIG, "gen2", config)

    def _set_generic_interrupt(self, reg_addr, kind, config):
        try:
            data = bytearray(11)
            data[0] = self._set_bits(
                data[0], _BMA400_INT_AXES_EN_MSK, _BMA400_INT_AXES_EN_POS, config.get("axes_sel", 0)
            )
            data[0] = self._set_bits(
                data[0], _BMA400_INT_DATA_SRC_MSK, _BMA400_INT_DATA_SRC_POS, config.get("data_src", 0)
            )
            data[0] = self._set_bits(
                data[0], _BMA400_INT_REFU_MSK, _BMA400_INT_REFU_POS, config.get("ref_update", 0)
            )
            data[0] = (data[0] & ~_BMA400_INT_HYST_MSK & 0xFF) | (
                config.get("hysteresis", 0) & _BMA400_INT_HYST_MSK
            )
            data[1] = self._set_bits(
                data[1], _BMA400_GEN_INT_CRITERION_MSK, _BMA400_GEN_INT_CRITERION_POS,
                config.get("criterion_sel", 0),
            )
            data[1] = (data[1] & ~_BMA400_GEN_INT_COMB_MSK & 0xFF) | (
                config.get("evaluate_axes", 0) & _BMA400_GEN_INT_COMB_MSK
            )
            data[2] = config.get("gen_int_thres", 0) & 0xFF

            gen_int_dur = config.get("gen_int_dur", 0)
            data[3] = (gen_int_dur >> 8) & 0xFF
            data[4] = gen_int_dur & 0xFF

            ref_update = config.get("ref_update", 0)
            if ref_update == BMA400_UPDATE_MANUAL:
                for i, key in enumerate(("int_thres_ref_x", "int_thres_ref_y", "int_thres_ref_z")):
                    value = config.get(key, 0)
                    data[5 + i * 2] = value & 0xFF
                    data[6 + i * 2] = (value >> 8) & 0xFF
                self._set_regs(reg_addr, data)
            else:
                self._set_regs(reg_addr, data[0:5])

            self._map_int_pin(kind, config.get("int_chan", BMA400_UNMAP_INT_PIN))
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def set_orientation_change_interrupt(self, config):
        """
        Set the orientation change interrupt configuration.

        :param config: dict with keys axes_sel, data_src, ref_update,
                        orient_thres, stability_thres, orient_int_dur,
                        orient_ref_x/y/z, int_chan
        """
        try:
            data = bytearray(10)
            data[0] = self._set_bits(
                data[0], _BMA400_INT_AXES_EN_MSK, _BMA400_INT_AXES_EN_POS, config.get("axes_sel", 0)
            )
            data[0] = self._set_bits(
                data[0], _BMA400_INT_DATA_SRC_MSK, _BMA400_INT_DATA_SRC_POS, config.get("data_src", 0)
            )
            data[0] = self._set_bits(
                data[0], _BMA400_INT_REFU_MSK, _BMA400_INT_REFU_POS, config.get("ref_update", 0)
            )
            data[1] = config.get("orient_thres", 0) & 0xFF
            data[2] = config.get("stability_thres", 0) & 0xFF
            data[3] = config.get("orient_int_dur", 0) & 0xFF

            ref_update = config.get("ref_update", 0)
            if ref_update == BMA400_UPDATE_MANUAL:
                for i, key in enumerate(("orient_ref_x", "orient_ref_y", "orient_ref_z")):
                    value = config.get(key, 0)
                    data[4 + i * 2] = value & 0xFF
                    data[5 + i * 2] = (value >> 8) & 0xFF
                self._set_regs(BMA400_REG_ORIENTCH_INT_CONFIG, data)
            else:
                self._set_regs(BMA400_REG_ORIENTCH_INT_CONFIG, data[0:4])

            self._map_int_pin("orient", config.get("int_chan", BMA400_UNMAP_INT_PIN))
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def set_tap_interrupt(self, config):
        """
        Set the tap detection interrupt configuration.

        :param config: dict with keys axes_sel, sensitivity, tics_th, quiet,
                        quiet_dt, int_chan
        """
        try:
            data = bytearray(self._get_regs(BMA400_REG_TAP_CONFIG, 2))
            data[0] = self._set_bits(
                data[0], _BMA400_TAP_AXES_EN_MSK, _BMA400_TAP_AXES_EN_POS, config.get("axes_sel", 0)
            )
            data[0] = (data[0] & ~_BMA400_TAP_SENSITIVITY_MSK & 0xFF) | (
                config.get("sensitivity", 0) & _BMA400_TAP_SENSITIVITY_MSK
            )
            data[1] = self._set_bits(
                data[1], _BMA400_TAP_QUIET_DT_MSK, _BMA400_TAP_QUIET_DT_POS, config.get("quiet_dt", 0)
            )
            data[1] = self._set_bits(
                data[1], _BMA400_TAP_QUIET_MSK, _BMA400_TAP_QUIET_POS, config.get("quiet", 0)
            )
            data[1] = (data[1] & ~_BMA400_TAP_TICS_TH_MSK & 0xFF) | (
                config.get("tics_th", 0) & _BMA400_TAP_TICS_TH_MSK
            )
            self._set_regs(BMA400_REG_TAP_CONFIG, data)
            self._map_int_pin("tap", config.get("int_chan", BMA400_UNMAP_INT_PIN))
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def set_step_counter_interrupt(self, config):
        """
        Set the step counter interrupt pin mapping.

        The step counter itself needs no configuration beyond enabling it
        with enable_interrupt(BMA400_STEP_COUNTER_INT_EN, True); this only
        controls which pin (if any) it interrupts on.

        :param config: dict with key int_chan
        """
        try:
            self._map_int_pin("step", config.get("int_chan", BMA400_UNMAP_INT_PIN))
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def set_activity_change_interrupt(self, config):
        """
        Set the activity change interrupt configuration.

        :param config: dict with keys act_ch_thres, axes_sel, data_source,
                        act_ch_ntps, int_chan
        """
        try:
            data = bytearray(2)
            data[0] = config.get("act_ch_thres", 0) & 0xFF
            data[1] = self._set_bits(
                data[1], _BMA400_ACT_CH_AXES_EN_MSK, _BMA400_ACT_CH_AXES_EN_POS, config.get("axes_sel", 0)
            )
            data[1] = self._set_bits(
                data[1], _BMA400_ACT_CH_DATA_SRC_MSK, _BMA400_ACT_CH_DATA_SRC_POS,
                config.get("data_source", 0),
            )
            data[1] = (data[1] & ~_BMA400_ACT_CH_NPTS_MSK & 0xFF) | (
                config.get("act_ch_ntps", 0) & _BMA400_ACT_CH_NPTS_MSK
            )
            self._set_regs(BMA400_REG_ACT_CH_CONFIG_0, data)
            self._map_int_pin("act_ch", config.get("int_chan", BMA400_UNMAP_INT_PIN))
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def set_wakeup_interrupt(self, config):
        """
        Set the wakeup interrupt configuration.

        :param config: dict with keys wakeup_ref_update, sample_count,
                        wakeup_axes_en, int_wkup_threshold, int_wkup_ref_x/y/z,
                        int_chan
        """
        try:
            data = bytearray(5)
            data[0] = (data[0] & ~_BMA400_WKUP_REF_UPDATE_MSK & 0xFF) | (
                config.get("wakeup_ref_update", 0) & _BMA400_WKUP_REF_UPDATE_MSK
            )
            data[0] = self._set_bits(
                data[0], _BMA400_SAMPLE_COUNT_MSK, _BMA400_SAMPLE_COUNT_POS, config.get("sample_count", 0)
            )
            data[0] = self._set_bits(
                data[0], _BMA400_WAKEUP_EN_AXES_MSK, _BMA400_WAKEUP_EN_AXES_POS,
                config.get("wakeup_axes_en", 0),
            )
            data[1] = config.get("int_wkup_threshold", 0) & 0xFF
            data[2] = config.get("int_wkup_ref_x", 0) & 0xFF
            data[3] = config.get("int_wkup_ref_y", 0) & 0xFF
            data[4] = config.get("int_wkup_ref_z", 0) & 0xFF
            self._set_regs(BMA400_REG_WAKEUP_INT_CONF_0, data)
            self._map_int_pin("wakeup", config.get("int_chan", BMA400_UNMAP_INT_PIN))
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def set_auto_wakeup(self, config):
        """
        Set the auto-wakeup timeout configuration.

        :param config: dict with keys wakeup_timeout (BMA400_ENABLE/_DISABLE)
                        and timeout_thres (12 bit, 2.5ms/LSB)
        """
        try:
            reg_data = self._get_regs(BMA400_REG_AUTOWAKEUP_1, 1)[0]
            reg_data = self._set_bits(
                reg_data, _BMA400_WAKEUP_TIMEOUT_MSK, _BMA400_WAKEUP_TIMEOUT_POS,
                config.get("wakeup_timeout", 0),
            )
            threshold = config.get("timeout_thres", 0)
            lsb = threshold & 0x0F
            msb = (threshold >> 4) & 0xFF
            reg_data = self._set_bits(
                reg_data, _BMA400_WAKEUP_TIMEOUT_THRES_MSK, _BMA400_WAKEUP_TIMEOUT_THRES_POS, lsb
            )
            self._set_regs(BMA400_REG_AUTOWAKEUP_0, [msb, reg_data])
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def set_auto_low_power(self, config):
        """
        Set the auto low-power mode configuration.

        :param config: dict with keys auto_low_power_trigger (a bitwise OR
                        of BMA400_AUTO_LP_* flags) and
                        auto_lp_timeout_threshold (12 bit, 2.5ms/LSB)
        """
        try:
            reg_data = self._get_regs(BMA400_REG_AUTO_LOW_POW_1, 1)[0]
            trigger = config.get("auto_low_power_trigger", 0)
            reg_data = (reg_data & ~_BMA400_AUTO_LOW_POW_MSK & 0xFF) | (
                trigger & _BMA400_AUTO_LOW_POW_MSK
            )

            if trigger & 0x0C:
                threshold = config.get("auto_lp_timeout_threshold", 0)
                msb = (threshold & _BMA400_AUTO_LP_THRES_MSK) >> _BMA400_AUTO_LP_THRES_POS
                lsb = threshold & _BMA400_AUTO_LP_THRES_LSB_MSK
                reg_data = self._set_bits(
                    reg_data, _BMA400_AUTO_LP_TIMEOUT_LSB_MSK, _BMA400_AUTO_LP_TIMEOUT_LSB_POS, lsb
                )
                self._set_regs(BMA400_REG_AUTO_LOW_POW_0, [msb])

            self._set_regs(BMA400_REG_AUTO_LOW_POW_1, [reg_data])
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    # ------------------------------------------------------------------
    # FIFO
    # ------------------------------------------------------------------

    @staticmethod
    def _bytes_per_fifo_frame(fifo_flags):
        num_axes = (
            bool(fifo_flags & BMA400_FIFO_X_EN)
            + bool(fifo_flags & BMA400_FIFO_Y_EN)
            + bool(fifo_flags & BMA400_FIFO_Z_EN)
        )
        bytes_per_axis = 1 if (fifo_flags & BMA400_FIFO_8_BIT_EN) else 2
        return 1 + num_axes * bytes_per_axis

    def set_fifo_config(self, config):
        """
        Set the FIFO configuration.

        :param config: dict with keys conf_regs (bitwise OR of BMA400_FIFO_*
                        flags), conf_status (BMA400_ENABLE/_DISABLE),
                        fifo_watermark (in number of measurements),
                        fifo_full_channel, fifo_wm_channel
        """
        try:
            conf_regs = config.get("conf_regs", 0)
            sens_data = self._get_regs(BMA400_REG_FIFO_CONFIG_0, 3)

            new_conf0 = conf_regs
            if config.get("conf_status", BMA400_ENABLE) == BMA400_DISABLE:
                new_conf0 = sens_data[0] & (~conf_regs & 0xFF)

            watermark = config.get("fifo_watermark", 0) * self._bytes_per_fifo_frame(conf_regs)
            wm_lsb = watermark & 0xFF
            wm_msb = (watermark >> 8) & _BMA400_FIFO_BYTES_CNT_MSK

            if wm_lsb == sens_data[1] and wm_msb == sens_data[2]:
                self._set_regs(BMA400_REG_FIFO_CONFIG_0, [new_conf0])
            else:
                self._set_regs(BMA400_REG_FIFO_CONFIG_0, [new_conf0, wm_lsb, wm_msb])

            self._map_int_pin("fifo_wm", config.get("fifo_wm_channel", BMA400_UNMAP_INT_PIN))
            self._map_int_pin("fifo_full", config.get("fifo_full_channel", BMA400_UNMAP_INT_PIN))
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    def get_fifo_length(self):
        """
        Get the number of data samples stored in the FIFO buffer.

        :return: Number of buffered samples, None on error
        """
        try:
            data = self._get_regs(BMA400_REG_FIFO_LENGTH, 2)
            num_bytes = data[0] | ((data[1] & _BMA400_FIFO_BYTES_CNT_MSK) << 8)
            conf_regs = self._get_regs(BMA400_REG_FIFO_CONFIG_0, 1)[0]
            self.status = BMA400_OK
            return num_bytes // self._bytes_per_fifo_frame(conf_regs)
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL
            return None

    def _read_fifo_raw(self, length):
        """Read `length` raw bytes out of the FIFO, handling the read-enable gate."""
        reg_data = self._get_regs(BMA400_REG_FIFO_READ_EN, 1)[0]
        if reg_data == 0:
            return self._get_regs(BMA400_REG_FIFO_DATA, length)

        self._set_regs(BMA400_REG_FIFO_READ_EN, [0])
        time.sleep_us(1000)
        data = self._get_regs(BMA400_REG_FIFO_DATA, length)
        self._set_regs(BMA400_REG_FIFO_READ_EN, [1])
        return data

    def get_fifo_data(self, num_data):
        """
        Read accelerometer data out of the FIFO buffer, converted to g's.

        :param num_data: Number of samples to read
        :return: List of BMA400Data instances (shorter than num_data if the
                 FIFO held less data than requested), None on error
        """
        try:
            conf_regs = self._get_regs(BMA400_REG_FIFO_CONFIG_0, 1)[0]
            fifo_length = self.get_fifo_length()
            if fifo_length is None:
                return None

            requested = min(num_data, fifo_length)
            num_bytes = requested * self._bytes_per_fifo_frame(conf_regs)
            if conf_regs & BMA400_FIFO_TIME_EN:
                num_bytes += _BMA400_FIFO_BYTES_OVERREAD

            raw = self._read_fifo_raw(num_bytes)
            frames, sensor_time = self._unpack_accel_frames(raw, conf_regs, requested)

            range_ = self._get_accel_conf_raw()["range"]
            g_range = 2 << range_
            # unpack_accel() (see _unpack_accel_frames) always left-shifts an
            # 8 bit FIFO sample by 4 bits before sign-extending it, so every
            # frame it hands back is on the same 12 bit scale regardless of
            # BMA400_FIFO_8_BIT_EN. The conversion factor is therefore fixed.
            raw_to_g = g_range / 2048.0

            result = []
            for x, y, z in frames:
                sample = BMA400Data()
                sample.accel_x = x * raw_to_g
                sample.accel_y = y * raw_to_g
                sample.accel_z = z * raw_to_g
                sample.sensor_time_ms = sensor_time * 1000 // 25600
                result.append(sample)

            self.status = BMA400_OK
            return result
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL
            return None

    def _unpack_accel_frames(self, data, conf_regs, frame_count):
        """
        Parse accelerometer frames out of a raw FIFO byte buffer.

        Direct port of bma400.c's unpack_accel_frame()/unpack_accel(). Header
        bytes select which axes are present in each frame and whether the
        buffer holds an accelerometer, sensor-time, control or empty frame.

        :return: Tuple of (list of (x, y, z) raw counts, fifo sensor time)
        """
        frames = []
        sensor_time = 0
        length = len(data)
        index = 0

        while index < length:
            frame_header = data[index]
            accel_width = 1 if (frame_header & BMA400_FIFO_8_BIT_EN) else 0
            frame_header &= _BMA400_AWIDTH_MASK
            index += 1

            if frame_header == _BMA400_FIFO_EMPTY_FRAME:
                break

            if frame_header == _BMA400_FIFO_SENSOR_TIME:
                if index + 3 > length:
                    break
                sensor_time = data[index] | (data[index + 1] << 8) | (data[index + 2] << 16)
                index += 3
                continue

            if frame_header == _BMA400_FIFO_CONTROL_FRAME:
                if index + 1 > length:
                    break
                index += 1
                continue

            axes_headers = (
                _BMA400_FIFO_XYZ_ENABLE,
                _BMA400_FIFO_X_ENABLE,
                _BMA400_FIFO_Y_ENABLE,
                _BMA400_FIFO_Z_ENABLE,
                _BMA400_FIFO_XY_ENABLE,
                _BMA400_FIFO_YZ_ENABLE,
                _BMA400_FIFO_XZ_ENABLE,
            )
            if frame_header not in axes_headers:
                break

            num_axes = bin(frame_header & _BMA400_FIFO_DATA_EN_MASK).count("1")
            bytes_needed = num_axes * (2 if accel_width == 0 else 1)
            if index + bytes_needed > length:
                break

            axis_bits = frame_header & _BMA400_FIFO_DATA_EN_MASK
            values = [0, 0, 0]
            for axis in range(3):
                axis_mask = 0x02 << axis  # X=0x02, Y=0x04, Z=0x08
                if axis_bits & axis_mask:
                    if accel_width == 0:
                        lsb = data[index]
                        msb = data[index + 1]
                        index += 2
                        values[axis] = _s12(((msb << 4) | lsb) & 0x0FFF)
                    else:
                        msb = data[index]
                        index += 1
                        values[axis] = _s12((msb << 4) & 0x0FFF)

            frames.append(tuple(values))
            if len(frames) == frame_count:
                break

        return frames, sensor_time

    def flush_fifo(self):
        """Clear all data in the FIFO buffer."""
        try:
            self._set_regs(BMA400_REG_COMMAND, [BMA400_FIFO_FLUSH_CMD])
            self.status = BMA400_OK
        except OSError:
            self.intf_rslt = BMA400_E_COM_FAIL
            self.status = BMA400_E_COM_FAIL

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def check_status(self):
        """
        Check whether an error or a warning has occurred.

        :return: BMA400_ERROR, BMA400_WARNING or BMA400_OK
        """
        if self.status < BMA400_OK:
            return BMA400_ERROR

        if self.status > BMA400_OK:
            return BMA400_WARNING

        return BMA400_OK

    def status_string(self):
        """
        Get a short description of the current status code.

        :return: Description of the status, an empty string when it is OK
        """
        return _STATUS_STRINGS.get(self.status, "Undefined error code")
