#ifndef ROS_H_
#define ROS_H_

#define USE_USBCON

#include <ArduinoHardware.h>
#include <ros/node_handle.h>

namespace ros
{
  // typedef NodeHandle_<ArduinoHardware, num_pub, num_sub, input_buffer_size, output_buffer_size> NodeHandle;
  typedef NodeHandle_<ArduinoHardware, 4, 4, 128, 384> NodeHandle;
}

#endif
