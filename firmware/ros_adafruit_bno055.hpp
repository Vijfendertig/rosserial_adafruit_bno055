#ifndef __ROS_ADAFRUIT_BNO055__
#define __ROS_ADAFRUIT_BNO055__


#include "arduino_micro_ros.h"
#include <std_msgs/Bool.h>
#include <ros_adafruit_bno055/Imu.h>
#include <ros_adafruit_bno055/CalibrationStatus.h>
#include <Adafruit_BNO055.h>


namespace ros_adafruit_bno055 {

  class RosAdafruitBNO055 {
    private:  // Data types and member variables.
      ros::NodeHandle * node_handle_;
      Adafruit_BNO055 sensor_;
      Imu measurements_message_;
      CalibrationStatus calibration_status_message_;
      ros::Subscriber<std_msgs::Bool, RosAdafruitBNO055> enable_subscriber_;
      ros::Publisher measurements_publisher_;
      ros::Publisher calibration_status_publisher_;
      bool enable_;
      unsigned long int measurements_publish_interval_;
      unsigned long int measurements_last_published_;
      unsigned long int calibration_status_publish_interval_;
      unsigned long int calibration_status_last_published_;
      struct StoredCalibrationData {
        // Add a valid field before and after the calibration data to detect interrupted writes.
        bool valid_front;
        adafruit_bno055_offsets_t data;
        ros::Time timestamp;
        bool valid_rear;
      };
      static constexpr uint16_t calibration_slots_address_ = 0U;
      static constexpr int8_t calibration_slots_count_ = 8U;
      int8_t current_calibration_slot_;
    public:  // Member functions.
      RosAdafruitBNO055(ros::NodeHandle * node_handle, unsigned long int measurements_publish_interval, unsigned long int calibration_status_publish_interval);
      ~RosAdafruitBNO055() = default;
      void setup();
      void enable();
      void disable();
      void spinOnce();
    private:  // Member functions.
      void enableCallback(const std_msgs::Bool & message);
      void getAndPublishMeasurements();
      void getAndPublishCalibrationStatus();
      void resetStoredCalibrationData(StoredCalibrationData & data);
      void loadCalibrationFromEeprom();
      void saveCalibrationToEeprom();
  };
  
}


#endif
