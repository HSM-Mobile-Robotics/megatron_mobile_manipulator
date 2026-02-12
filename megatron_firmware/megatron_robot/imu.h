/*

MIT License
Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab

Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

*/

#ifndef _IMU_H_
#define _IMU_H_

#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

// IMU calibration structure
typedef struct {
  const float gyro_scaling_factor;
  const float *gyro_offset;
  const float accel_scaling_factor;
  const float *accel_bias;
  const float (*accel_a_matrix_inverse)[3];
  const float *linear_accel_covariance;
  const float *angular_vel_covariance;
  const float *orientation_covariance;
} ImuCalibration;

class IMU {
private:
  ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object

  const ImuCalibration *calibration;

public:
  IMU(const ImuCalibration *calib);
  void imuInit(uint8_t csPin, SPIClass &spiPort, uint32_t spiFreq);
  void updateData(sensor_msgs__msg__Imu *imu_msg);
};

#endif
