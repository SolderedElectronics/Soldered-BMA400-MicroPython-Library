# Soldered BMA400 Accelerometer MicroPython Library

| ![Soldered BMA400 Accelerometer breakout](TODO_PRODUCT_IMAGE_URL) |
| :-----------------------------------------------------------------------------------------------: |
|                          [Soldered BMA400 Accelerometer breakout](https://www.solde.red/333419)                     |

<!-- TODO: product not released yet (SKU 333419), swap the image URL above once the listing is live -->

Breakout board for the Bosch BMA400 ultra-low-power triaxial accelerometer, drawing as little as 14 µA in normal mode. It measures 3-axis acceleration across four selectable ranges (±2g to ±16g) at up to 800 Hz, and includes on-chip step counting, tap detection, orientation and activity-change recognition, and a FIFO buffer. The board communicates over I2C only and is part of the [Qwiic ecosystem](https://soldered.com/collections/qwiic-ecosystem).

### Quick start

```python
from bma400 import BMA400, BMA400_OK
import time

sensor = BMA400()  # Or BMA400(address=BMA400_I2C_ADDRESS_SDO_HIGH)

while True:
    if sensor.get_sensor_data() == BMA400_OK:
        print(sensor.data.accel_x, sensor.data.accel_y, sensor.data.accel_z)
    time.sleep(1)
```

Have a look at the scripts in `Examples/` for basic readings, motion detection and the built-in step counter.

### How to install

Use [mim](https://checkmim.com/packages).

or

After [**installing the mpremote package**](https://docs.micropython.org/en/latest/reference/mpremote.html), install the library on your board using the following command:

```sh
  mpremote mip install github:SolderedElectronics/Soldered-BMA400-MicroPython-Library
```
Or, if you're running a Windows OS:

```sh
  python -m mpremote mip install github:SolderedElectronics/Soldered-BMA400-MicroPython-Library
```

### Repository Contents

- **bma400.py** - MicroPython driver class, I2C only
- **package.json** - mip install manifest
- **/Examples** - examples for basic readings, motion detection and the built-in step counter

### Examples

| Example | What it does |
| :------ | :----------- |
| `bma400-basicReadings.py` | Reads X, Y, Z acceleration in a loop, the mode most applications want |
| `bma400-motionDetection.py` | Configures the generic interrupt feature and reacts to it on a hardware interrupt pin |
| `bma400-stepCounter.py` | Uses the built-in step counter and activity type detection (running/walking/still) |

### Hardware design

You can find hardware design for this board in _Soldered BMA400 Accelerometer breakout_ hardware repository.

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

![Soldered Logo](https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png)

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Original source

This library is a port of the [Soldered BMA400 Arduino library](https://github.com/SolderedElectronics/Soldered-BMA400-Arduino-Library), which wraps [SparkFun's BMA400 Arduino Library](https://github.com/sparkfun/SparkFun_BMA400_Arduino_Library), itself built on the [BMA400 Sensor API](https://github.com/BoschSensortec/BMA400-Sensor-API) by Bosch Sensortec. Thank you, SparkFun and Bosch Sensortec.

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
