This directory contains Device Tree overlays for various boards.

# Raspberry Pi

Add the following line to `/boot/config.txt`:
```
dtoverlay=rpi-hvac
```

The `dtparam=i2c_arm=on` option does *not* need to be uncommented. According to
the documentation, the I2C bus is enabled automatically whenever an I2C device
is attached to the bus by a Device Tree overlay.

Reference:
[Raspberry Pi Documentation - Configuration | Device Tree Overlays](https://www.raspberrypi.com/documentation/computers/configuration.html#part2)
