# SMA13x-Sensor-API and Sensor Driver

## Table of Contents
 - [Introduction](#Intro)
 - [License](#License)
 - [Sensor interfaces](#interfaces)
 - [Architecture](#Architecture)
 - [Operation examples](#examples)

## Introduction <a name=Intro></a>


## License <a name=License></a>
See [LICENSE](LICENSE.md) file

## Sensor interfaces <a name=interfaces></a>
* I2C
* SPI

## Architecture <a name=Architecture></a>
```
                  User space
-------------------------------------------------------
                 |          |
               sysfs       dev
                 \          /
               input-subsystem
	             |
sensor_API <-- sma13x_driver --> sma13x_SPI/I2C_driver
                                           |
                                      SPI/I2C_bus
                                           |
-------------------------------------------------------
                  Hardware
```
## Operation examples <a name=examples></a>
1. Userspace
The driver exposes a device file node under /dev/input/event*, which can be read as a normal Linux file. Tools like evtest can also be used for read data out. Eg.:
```
sudo evtest /dev/input/event0
```
The data will be displayed on the console with timestamp.

2. Sysfs
The driver also exposes a set of sysfs nodes under /sys/devices/virtual/input/input*, where users can get information about the sensor and also control the sensor. Eg.:

```

# read the chip id
cat /sys/devices/virtual/input/input0/chip_id

# read the asynced acc data 
cat /sys/devices/virtual/input/input0/value

# read the acc power config 
cat /sys/devices/virtual/input/input0/pw_cfg

# set the acc power to normal mode
echo 0 > /sys/devices/virtual/input/input0/pw_cfg

# set the acc power to suspend mode
echo 3 > /sys/devices/virtual/input/input0/pw_cfg
```
