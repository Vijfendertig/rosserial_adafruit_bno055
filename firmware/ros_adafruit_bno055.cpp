#include <Arduino.h>
#include <EEPROM.h>

#include "ros_adafruit_bno055.hpp"


namespace ros_adafruit_bno055 {

  RosAdafruitBNO055::RosAdafruitBNO055(ros::NodeHandle * node_handle, unsigned long int measurements_publish_interval, unsigned long int calibration_status_publish_interval):
    node_handle_{node_handle},
    sensor_{},
    enable_subscriber_{"/bno055/enable", &RosAdafruitBNO055::enableCallback, this},
    measurements_publisher_{"bno055/imu", &measurements_message_},
    calibration_status_publisher_{"bno055/calib_status", &calibration_status_message_},
    enable_{false},
    measurements_publish_interval_{measurements_publish_interval},
    calibration_status_publish_interval_{calibration_status_publish_interval},
    measurements_last_published_{0UL},
    calibration_status_last_published_{0UL},
    current_calibration_slot_{calibration_slots_count_ - 1}
  {
    measurements_message_.header.frame_id = "bno055";
    measurements_message_.header.seq = 0;
    calibration_status_message_.last_saved = ros::Time();
    pinMode(LED_BUILTIN, OUTPUT);
  }


  void RosAdafruitBNO055::setup() {
    if(sensor_.begin()) {
      loadCalibrationFromEeprom();
      node_handle_->advertise(measurements_publisher_);
      node_handle_->advertise(calibration_status_publisher_);
      node_handle_->subscribe(enable_subscriber_);
    }
    else {
      while(true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
      }
    }
  }

  void RosAdafruitBNO055::enable() {
    if(enable_ == false) {
      measurements_last_published_ = millis() - measurements_publish_interval_;
      calibration_status_last_published_ = millis() - calibration_status_publish_interval_;
    }
    enable_ = true;
  }


  void RosAdafruitBNO055::disable() {
    if(enable_ == true) {
      saveCalibrationToEeprom();
    }
    enable_ = false;
  }


  void RosAdafruitBNO055::spinOnce() {
    if(enable_) {
      auto current = millis();
      if(current >= measurements_last_published_ + measurements_publish_interval_
          || current < measurements_last_published_) {
        getAndPublishMeasurements();
        measurements_last_published_ += measurements_publish_interval_;    
      }
      if(current >= calibration_status_last_published_ + calibration_status_publish_interval_
          || current < calibration_status_last_published_) {
        digitalWrite(LED_BUILTIN, HIGH);
        getAndPublishCalibrationStatus();
        digitalWrite(LED_BUILTIN, LOW);
        calibration_status_last_published_ += calibration_status_publish_interval_;    
      }
    }
  }


  void RosAdafruitBNO055::enableCallback (const std_msgs::Bool & message) {
    if(message.data == true) {
      enable();
    }
    else {
      disable();
    }
  }


  void RosAdafruitBNO055::getAndPublishMeasurements() {
    // Store timestamp in the message's header.
    measurements_message_.header.stamp = node_handle_->now();
    // Get measurements.
    auto quaternion = sensor_.getQuat();
    auto angular_velocity = sensor_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    auto linear_acceleration = sensor_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // Store absolute orientation (as a quaternion).
    measurements_message_.orientation.x = quaternion.x();
    measurements_message_.orientation.y = quaternion.y();
    measurements_message_.orientation.z = quaternion.z();
    measurements_message_.orientation.w = quaternion.w();
    // Store ansular velocity (in rad/s);
    measurements_message_.angular_velocity.x = angular_velocity.x();
    measurements_message_.angular_velocity.y = angular_velocity.y();
    measurements_message_.angular_velocity.z = angular_velocity.z();
    // Store linear acceleration (in m/s^2).
    measurements_message_.linear_acceleration.x = linear_acceleration.x();
    measurements_message_.linear_acceleration.y = linear_acceleration.y();
    measurements_message_.linear_acceleration.z = linear_acceleration.z();
    // Publish message.
    measurements_publisher_.publish(&measurements_message_);
    // Augment messqge sequence id.
    ++ measurements_message_.header.seq;
  }


  void RosAdafruitBNO055::getAndPublishCalibrationStatus() {
    // Get calibration status.
    calibration_status_message_.system = 0;
    calibration_status_message_.accelerometer = 0;
    calibration_status_message_.gyroscope = 0;
    calibration_status_message_.magnetometer = 0;
    sensor_.getCalibration(&calibration_status_message_.system, &calibration_status_message_.gyroscope,
        &calibration_status_message_.accelerometer, &calibration_status_message_.magnetometer);
    // Publish message.
    calibration_status_publisher_.publish(&calibration_status_message_);
  }


  void RosAdafruitBNO055::resetStoredCalibrationData(StoredCalibrationData & data) {
    data.signature_front = 0xff;
    data.data.accel_offset_x = 0xffff;
    data.data.accel_offset_y = 0xffff;
    data.data.accel_offset_z = 0xffff;
    data.data.mag_offset_x = 0xffff;
    data.data.mag_offset_y = 0xffff;
    data.data.mag_offset_z = 0xffff;
    data.data.gyro_offset_x = 0xffff;
    data.data.gyro_offset_y = 0xffff;
    data.data.gyro_offset_z = 0xffff;
    data.data.accel_radius = 0xffff;
    data.data.mag_radius = 0xffff;
    data.timestamp = ros::Time(0xffffffff, 0xffffffff);
    data.signature_rear = 0xff;
  }


  void RosAdafruitBNO055::loadCalibrationFromEeprom() {
    int8_t preferred_calibration_slot{-1};
    ros::Time preferred_calibration_timestamp;
    StoredCalibrationData stored_calibration;
    for(int8_t calibration_slot = 0; calibration_slot < calibration_slots_count_; ++ calibration_slot) {
      EEPROM.get(calibration_slots_address_ + calibration_slot * sizeof(StoredCalibrationData), stored_calibration);
      if(stored_calibration.signature_front == calibration_signature_
         && stored_calibration.signature_rear == calibration_signature_
         && (stored_calibration.timestamp.sec > preferred_calibration_timestamp.sec
             || (stored_calibration.timestamp.sec == preferred_calibration_timestamp.sec
                 && stored_calibration.timestamp.nsec > preferred_calibration_timestamp.nsec))) {
        preferred_calibration_slot = calibration_slot;
        preferred_calibration_timestamp = stored_calibration.timestamp;
      }
    }
    if(preferred_calibration_slot != -1) {
      current_calibration_slot_ = preferred_calibration_slot;
      EEPROM.get(calibration_slots_address_ + preferred_calibration_slot * sizeof(StoredCalibrationData), stored_calibration);
      sensor_.setSensorOffsets(stored_calibration.data);
      calibration_status_message_.last_saved = stored_calibration.timestamp;
    }
  }


  void RosAdafruitBNO055::saveCalibrationToEeprom() {
    if(sensor_.isFullyCalibrated()) {
      // Get new calibration slot.
      uint8_t new_calibration_slot = (current_calibration_slot_ + 1) % calibration_slots_count_;
      // Create empty calibration message and write it to the calibration slot.
      StoredCalibrationData new_calibration;
      resetStoredCalibrationData(new_calibration);
      EEPROM.put(calibration_slots_address_ + new_calibration_slot * sizeof(StoredCalibrationData), new_calibration);
      // Get calibration from sensor and write it to the calibration slot.
      if(sensor_.getSensorOffsets(new_calibration.data)) {
        new_calibration.timestamp = node_handle_->now();
        new_calibration.signature_front = calibration_signature_;
        new_calibration.signature_rear = calibration_signature_;
        EEPROM.put(calibration_slots_address_ + new_calibration_slot * sizeof(StoredCalibrationData), new_calibration);
        current_calibration_slot_ = new_calibration_slot;
        calibration_status_message_.last_saved = new_calibration.timestamp;
      }
    }
  }
  
}
