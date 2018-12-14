cmake_minimum_required(VERSION 3.1)

# Compile as C++11.
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The Arduino CMake included with ROS Melodic Morenia assumes the hardware platform path (where the
# boards.txt file lives) is ${ARDUINO_SDK_PATH}/hardware/arduino. At least for my version 1.8.7,
# this should be ${ARDUINO_SDK_PATH}/hardware/arduino/avr.
register_hardware_platform(${ARDUINO_SDK_PATH}/hardware/arduino/avr)

include_directories(
  ${ARDUINO_SDK_PATH}/hardware/arduino/avr/libraries
  ${ARDUINO_SDK_PATH}/hardware/arduino/avr/libraries/Wire/src
  ${ARDUINO_SDK_PATH}/hardware/arduino/avr/libraries/EEPROM/src
  ${ROS_LIB_DIR}
)

add_definitions(-DUSB_CON)

link_directories(
  ${ARDUINO_SDK_PATH}/hardware/arduino/avr/libraries
  ${CMAKE_CURRENT_SOURCE_DIR}/libraries
)

generate_arduino_firmware(arduino_micro
  SRCS arduino_micro_imu_node.cpp
       ros_adafruit_bno055.cpp
       ${ARDUINO_SDK_PATH}/hardware/arduino/avr/libraries/Wire/src/Wire.cpp
       ${ARDUINO_SDK_PATH}/hardware/arduino/avr/libraries/Wire/src/utility/twi.c
       libraries/Adafruit_BNO055/Adafruit_BNO055.cpp
       ${ROS_LIB_DIR}/time.cpp
  ARDLIBS Wire EEPROM Adafruit_Sensor Adafruit_BNO055
  BOARD micro
  PROGRAMMER arduinoisp
  PORT /dev/ttyACM0
)
