# driver_pololu_maestro

## Overview

This package includes driver software for the Pololu [Maestro] family of UART servo controllers.

**Keywords:** pololu maestro servo driver

### License

The source code is released under a [MIT license](LICENSE).

**Author: Paul D'Angio<br />
Maintainer: Paul D'Angio, pcdangio@gmail.com**

The driver_pololu_maestro package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [serial](http://wiki.ros.org/serial) (ROS serial package)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

        cd catkin_workspace/src
        git clone https://github.com/pcdangio/ros-driver_pololu_maestro.git driver_pololu_maestro
        cd ../
        catkin_make

## Usage

Run the driver node with:

        rosrun driver_pololu_maestro node

## Nodes

### node

A driver for the Pololu [Maestro].


#### Published Topics
* **`servo_controller/position/channel_X`** ([driver_pololu_maestro/servo_position](https://github.com/pcdangio/ros-driver_pololu_maestro/blob/master/msg/servo_position.msg))

        Provides the current expected position of a servo in PWM microseconds.
        Multiple versions of this topic are published based on the specified channels in use on the Maestro.  See "channels" in the Parameters section below.
        These topics will only be published if specified in the node parameters.  See "publish_positions" in the Parameters section below.
        "channel_X" is the channel that the servo is connected to on the Maestro, where X is the channel number.

#### Subscribed Topics

* **`servo_controller/set_target/channel_X`** ([driver_pololu_maestro/servo_target](https://github.com/pcdangio/ros-driver_pololu_maestro/blob/master/msg/servo_target.msg))

        Sets the target position, speed limit, and acceleration of a specified servo in PWM microseconds.
        Multiple versions of this topic are subscribed to based on the specified channels in use on the Maestro.  See "channels" in the Parameters section below.
        "channel_X" is the channel that the servo is connected to on the Maestro, where X is the channel number.

#### Parameters

* **`~/serial_port`** (string, default: /dev/ttyTHS2)

        The serial port connected to the Maestro.

* **`~/baud_rate`** (uint32, default: 57600)

        The baud rate used to communicate with the Maestro.
        By default, the Maestro firmware is configured to auto-detect the baud rate being used here.

* **`~/crc_enabled`** (bool, default: false)

        Indicates if the Maestro firmware is configured to use CRC checking.

* **`~/update_rate`** (double, default: 30)

        The update rate of the node, in Hz.

* **`~/device_number`** (uint8, default: 12)

        The device number assigned to the Maestro.  The default value in the firmware is 12.

* **`~/channels`** (vector<uint8>, default: [0, 1, 2, 3, 4, 5])

        A list of the active channels on the Maestro. This specifies which channels have a servo connected on the Maestro hardware.
        The driver will only subscribe to servo_target messages for channels listed in this parameter.
        The driver will only publish servo_position messages for channels listed in this parameter.

* **`~/publish_positions`** (bool, default: false)

        Indicates if the driver should publish servo_position messages.

* **`~/pwm_period`** (uint16, default: 20)

        The period of the PWM signal that the Maestro is configured to use when communicating with servos, in milliseconds.
        This parameter is important for setting accurate speed and acceleration values.
        The default value in the firmware is 20ms.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-driver_pololu_maestro/issues).


[ROS]: http://www.ros.org
[Maestro]: https://www.pololu.com/product/1350
