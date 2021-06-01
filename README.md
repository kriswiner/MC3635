# MC3635
<b>mCube's ultra-low-power wake-on-motion 3-axis accelerometer</b>

Based on mCube's Arduino demo [driver](https://github.com/mcubemems/mCube_mc36xx_arduino_driver), this sketch is specific for the [MC3635](https://mcubemems.com/product/mc3635-3-axis-accelerometer/) 3-axis accelerometer. The sketch supports I2C (here used at 400 kHz) for communication. The sketch is written for the STM32L432 ([Ladybug](https://www.tindie.com/products/tleracorp/ladybug-stm32l432-development-board/)) development board but just about any MCU that supports I2C and hardware interrupts can be used with minimal, if any, modification. 

The board I designed exposes the I2C address pin so more than one of these sensors could be used on the same I2C bus. This is not supported in this sketch but could easily be done using the I2CDev API.

The sketch places the MC3635 in Sniff mode at 7 Hz (configurable) and triggers an interrupt to the Ladybug whenever the motion threshold and hysteresis conditions are met (also configurable). 

I measured the power usage of the MC3635 at 0.3 +/- 0.1 uA in Sniff mode, pretty much what the [data sheet](https://mcubemems.com/wp-content/uploads/2019/11/MC3635-Datasheet-APS-048-0044v1.7.pdf) claims. This ultra-low-power, coupled with the low BOM cost and availability of the MC3635, makes it a promising candidate as a wake-on-motion watchdog for wearable devices.

![MC3635](https://user-images.githubusercontent.com/6698410/120250955-c1c60480-c234-11eb-87ee-be98a667afe7.jpg)
