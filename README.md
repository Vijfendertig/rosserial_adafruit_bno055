# Rosserial IMU node (Arduino Micro with Adafruit BNO055)

This package builds a rosserial compatible USB IMU from an Arduino Micro and an Adafruit BNO055 IMU. The Arduino and the BNO055 breakout board communicate via I2C, so you'll have to connect the 5V, GND, SDA and SCL of the breakout board to your Arduino Micro (D2 = SDA, D3 = SCL).

The rosserial node provides two publishers:

- `/bno055/imu` with the IMU's measurements (at 50 Hz) and
- `/bno055/calib_status` with the IMU's calibration status (at 1 Hz).

Both publishers can be enabled or disabled by sending a `true` or `false` bool message to `/bno055/enable`.

When the IMU is disabled while it is fully calibrated, the calibration offsets are stored in the Arduino's EEPROM memory. If stored offsets are available, they are restored after a reset.

## Dependencies

- [ROS](http://www.ros.org/). I used Melodic Morenia on Ubuntu 18.04 LTS, but other versions might work too.
- [rosserial_arduino](http://wiki.ros.org/rosserial_arduino).
- [Arduino IDE](https://www.arduino.cc/en/Main/Software). I used version 1.8.8. Other versions might work too, but the one included in Ubuntu 18.04 LTS doesn't.
- [arduino-cmake](https://github.com/queezythegreat/arduino-cmake). I [forked the repository and added a patch](https://github.com/Vijfendertig/arduino-cmake) to use `avr-gcc` and `avr-g++` from the Arduino IDE rather than the one provided with Ubuntu 18.04 LTS.
- [Adafruit_BNO055](https://github.com/adafruit/Adafruit_BNO055). The angular velocity is measured in deg/s, although the documentation states that it is expressed in rad/s ([issue](https://github.com/adafruit/Adafruit_BNO055/issues/50)). Because [REP 103](www.ros.org/reps/rep-0103.html) specifies to use radians and the sensor can be set up in rad/s, I [forked the repository and added a patch](https://github.com/Vijfendertig/Adafruit_BNO055) to read the angular velocity in rad/s directly rather than converting it afterwards.
- [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor). Used by Adafruit_BNO055.

The (patched) Adafruit_BNO055, Adafruit_Sensor and (patched) arduino-cmake dependencies are included as git submodules.

## Building

Include the package in a ROS workspace. Both building (messages, firmware...) and uploading the firmware is done using catkin_make.

Due to some internal details of rosserial_arduino's make_libraries.py script, building the package isn't as straightforward as I would like it to be. The problem is that to create our custom messages in the Arduino ros_lib library, rosserial_arduino's make_libraries.py script needs to source the workspace's setup script, which isn't available until the build is finished. See [https://github.com/ros-drivers/rosserial/issues/239] for more details. 
The most elegant workaround I found is to exclude the firmware from the default catkin_make (or CMake) target and build it manually afterwards.

So, to build the package including the firmware for the Arduino Micro, run:

- `catkin_make -DARDUINO_SDK_PATH=/opt/arduino-1.8.8` (to build everything except the firmware)
- `. ./devel/setup.bash` (or the setup script for your favourite shell)
- `catkin_make rosserial_adafruit_bno055_firmware_arduino_micro` (to build the firmware)
- `catkin_make rosserial_adafruit_bno055_firmware_arduino_micro-upload` (to upload the firmware to your Arduino Micro)

## Running

Just source the workspace's setup script and run `rosrun rosserial_python serial_node.py /dev/ttyACM0`. Start the `/bno055/imu` and `/bno055/calib_status` publishers by sending a `std_msgs/Bool` `true` message to the `/bno055/enable` subscriber. The `imu_publisher_node` subscribes to the compact rosserial_adafruit_bno055/Imu messages and publishes full sensor_msgs/Imu messages (including covariances).

There is also a `rosserial_adafruit_bno055.launch` file that launches both the rosserial node and a republisher node and sends an enable command to the IMU node. The launch file accepts two parameters: `bno055_port` which specifies the IMU node's device and `bno055_frame_id` which specifies the frame_id used in the full sensor_msgs/Imu message.

The calibration status of the system, accelerometer, gyroscope and magnetometer is given with integers from 0 to 3, where 0 means uncalibrated and 3 means fully calibrated.

To visualize the orientation vector, you can use rqt's pose view plugin. Just open the plugin (`Plugins`, `Visualization`, `Pose View`) and the topic monitor (`Plugins`, `Topics`, `Topic Monitor`) and drag the `/bno055/imu` topic (if you are running the republisher node too, you can also use the `/imu` topic) from the topic monitor to the pose view.

## License

MIT license, see LICENSE.md for details.

Git submodules:

- [arduino-cmake](https://github.com/queezythegreat/arduino-cmake): Unknown
- [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor): Apache License, Version 2.0
- [Adafruit_BNO055](https://github.com/adafruit/Adafruit_BNO055): MIT license