# MC3635
mCube's ultra-low-power wake-on-motion 3-axis accelerometer

Modification of mCube's Arduino demo sketch specfically for the MC3635 using I2C for communication. The sketch is written for the STM32L432 (Ladybug) development board but just about any MCU that supports I2C and hardware interrupts can be used with minimal, if any, modification. The MC3635 is placed in Sniff mode at 7 Hz (configurable) and triggers an interrupt whenever the motion threshold and hysteresis conditions are met (also configurable). I measured the power usage of the MC3635 at 0.3 +/- 0.1 uA in Sniff mode. This ultra-low-power, coupled with the low BOM cost and availability of the MC3635 makes it a promising candidate as a wake-on-motion watchdog for wearable devices.

![MC3635](https://user-images.githubusercontent.com/6698410/120250955-c1c60480-c234-11eb-87ee-be98a667afe7.jpg)
