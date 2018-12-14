# Rosserial IMU node (Arduino Micro with Adafruit BNO055)

This package builds a rosserial compatible USB IMU from an Arduino Micro and an Adafruit BNO055 IMU. The Arduino and the BNO055 breakout board communicate via I2C, so you'll have to connect the 5V, GND, SDA and SCL of the breakout board to your Arduino Micro (D2 = SDA, D3 = SCL).

The rosserial node provides two publishers:

- `/bno055/imu` with the IMU's measurements (at 50 Hz) and
- `/bno055/calib_status` with the IMU's calibration status (at 1 Hz).

Both publishers can be enabled or disabled by sending a `true` or `false` bool message to `/bno055/enable`.

When the IMU is disabled while it is fully calibrated, the calibration offsets are stored in the Arduino's EEPROM memory. If stored offsets are available, they are restored after a reset.

## Building

Due to a bug in rosserial_arduino [https://github.com/ros-drivers/rosserial/issues/239], building the package isn't as straightforward as I would like it to be. The problem is that to build the Arduino firmware, rosserial_arduino's make_libraries.py script needs to source the workspace's setup script, which isn't possible until the build is finished. The most elegant workaround I found is to exclude the firmware from the default catkin_make (or CMake) target and build it manually afterwards.

So, to build the package including the firmware fot the Arduino Micro, run:

- `catkin_make` (to build everything except the firmware)
- `. ./devel/setup.bash` (or the setup script for your favourite shell)
- `catkin_make ros_adafruit_bno055_firmware_arduino_micro` (to build the firmware)
- `catkin_make ros_adafruit_bno055_firmware_arduino_micro-upload` (to upload the firmware to your Arduino Micro)

## Running

Just source the workspace's setup script and run `rosrun rosserial_python serial_node.py /dev/ttyACM0`. Start the `/bno055/imu` and `/bno055/calib_status` publishers by sending a `std_msgs/Bool` `true` message to the `/bno055/enable` subscriber.

## License

MIT