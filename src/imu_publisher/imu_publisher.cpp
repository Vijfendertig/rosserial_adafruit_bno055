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


#include "imu_publisher.hpp"
#include <sensor_msgs/Imu.h>


namespace ros_adafruit_bno055 {

  ImuPublisher::ImuPublisher(const std::string & frame_id):
    frame_id_{frame_id},
    subscriber_compact_imu_{node_handle_.subscribe("/bno055/imu", 16, &ImuPublisher::compactImuCallback, this)},
    subscriber_calibration_status_{node_handle_.subscribe("/bno055/calib_status", 16, 
                                                          &ImuPublisher::calibrationStatusCallback, this)},
    publisher_full_imu_{node_handle_.advertise<sensor_msgs::Imu>("/imu", 16)}
  {
    // Reset cached calibration status.
    cached_calibration_status_.system = 0;
    cached_calibration_status_.accelerometer = 0;
    cached_calibration_status_.gyroscope = 0;
    cached_calibration_status_.magnetometer = 0;
    cached_calibration_status_.last_saved = ros::Time();
  }

  void ImuPublisher::compactImuCallback(const Imu::ConstPtr & compact_message) {
    sensor_msgs::Imu full_message;
    full_message.header.seq = compact_message->header.seq;
    full_message.header.stamp = compact_message->header.stamp;
    full_message.header.frame_id = compact_message->header.frame_id;
    full_message.orientation = compact_message->orientation;
    full_message.orientation_covariance[0] = -1;  // TODO: add covariance matrix.
    full_message.angular_velocity = compact_message->angular_velocity;
    full_message.angular_velocity_covariance[0] = -1;  // TODO: add covariance matrix.
    full_message.linear_acceleration = compact_message->linear_acceleration;
    full_message.linear_acceleration_covariance[0] = -1;  // TODO: add covariance matrix.
    publisher_full_imu_.publish(full_message);
  }


  void ImuPublisher::calibrationStatusCallback(const CalibrationStatus::ConstPtr & message) {
    cached_calibration_status_ = *message;
  }

}