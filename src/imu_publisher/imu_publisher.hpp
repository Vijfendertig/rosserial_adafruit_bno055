//! Imu (re)publisher for Arduino Micro Adafruit BNO055 rosserial node.
/*!
 *  This node republishes the compact (excluding covariances) adafruit_bno055/Imu message from the Arduino Micro
 *  Adafruit BNO055 rosserial node as standard sensor_msgs/Imu messages, taking into account the calibration status of
 *  the BNO055 sensor.
 * 
 *  \file
 * 
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __ROS_ADAFRUIT_BNO055__IMU_PUBLISHER__
#define __ROS_ADAFRUIT_BNO055__IMU_PUBLISHER__


#include <ros/ros.h>
#include <ros_adafruit_bno055/Imu.h>
#include <ros_adafruit_bno055/CalibrationStatus.h>


namespace ros_adafruit_bno055 {

  class ImuPublisher {
    private:
      ros::NodeHandle node_handle_;
      ros::Subscriber subscriber_compact_imu_;
      ros::Subscriber subscriber_calibration_status_;
      ros::Publisher publisher_full_imu_;
      std::string frame_id_;
      ros_adafruit_bno055::CalibrationStatus cached_calibration_status_;
    public:
      ImuPublisher(const std::string & frame_id);
      ~ImuPublisher() = default;
      void compactImuCallback(const Imu::ConstPtr & message);
      void calibrationStatusCallback(const CalibrationStatus::ConstPtr & message);
  };

}


#endif