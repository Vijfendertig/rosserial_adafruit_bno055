/*
 * Arduino micro IMU node
 */


#include <Arduino.h>
#include "arduino_micro_ros.h"
#include "ros_adafruit_bno055.hpp"


ros::NodeHandle node_handle;
ros_adafruit_bno055::RosAdafruitBNO055 ros_sensor(&node_handle, 20UL, 1000UL);


void setup()
{
  node_handle.initNode();
  ros_sensor.setup();
}


void loop()
{
  node_handle.spinOnce();
  ros_sensor.spinOnce();
}
