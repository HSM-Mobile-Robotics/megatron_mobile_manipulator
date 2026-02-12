/*
 
 MIT License
 Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab

 Authors:Venkata Prashanth Uppalapati <venkataprashanth.u@gamil.com>

*/




#include "imu.h"

IMU::IMU(const ImuCalibration* calib) : calibration(calib) {}
void IMU::imuInit(uint8_t csPin, SPIClass& spiPort, uint32_t spiFreq){
    spiPort.begin();
    myICM.begin(csPin, spiPort, spiFreq);

    // Here we are doing a SW reset to make sure the device starts in a known state
    myICM.swReset();
    delay(250);

    // Now wake the sensor up
    myICM.sleep(false);
    myICM.lowPower(false);

    // The next few configuration functions accept a bit-mask of sensors for which the settings
    // should be applied.

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                        ICM_20948_Sample_Mode_Continuous);
}

void IMU::updateData(sensor_msgs__msg__Imu* imu_msg) {
    byte i;
    float temp[3], accVal[3];

    myICM.getAGMT();
    // apply gyro offsets (bias)

    imu_msg->angular_velocity.x = (double)(calibration->gyro_scaling_factor * (myICM.agmt.gyr.axes.x - calibration->gyro_offset[0]));
    imu_msg->angular_velocity.y = (double)(calibration->gyro_scaling_factor * (myICM.agmt.gyr.axes.y - calibration->gyro_offset[1]));
    imu_msg->angular_velocity.z = (double)(calibration->gyro_scaling_factor * (myICM.agmt.gyr.axes.z - calibration->gyro_offset[2]));

    accVal[0] = myICM.agmt.acc.axes.x;
    accVal[1] = myICM.agmt.acc.axes.y;
    accVal[2] = myICM.agmt.acc.axes.z;

    // apply accel offsets (bias)

    for (i = 0; i < 3; i++) temp[i] = (accVal[i] - calibration->accel_bias[i]);
    imu_msg->linear_acceleration.x =
        (double)(calibration->accel_scaling_factor *
                 (calibration->accel_a_matrix_inverse[0][0] * temp[0] + calibration->accel_a_matrix_inverse[0][1] * temp[1] +
                  calibration->accel_a_matrix_inverse[0][2] * temp[2]));
    imu_msg->linear_acceleration.y =
        (double)(calibration->accel_scaling_factor *
                 (calibration->accel_a_matrix_inverse[1][0] * temp[0] + calibration->accel_a_matrix_inverse[1][1] * temp[1] +
                  calibration->accel_a_matrix_inverse[1][2] * temp[2]));
    imu_msg->linear_acceleration.z =
        (double)(calibration->accel_scaling_factor *
                 (calibration->accel_a_matrix_inverse[2][0] * temp[0] + calibration->accel_a_matrix_inverse[2][1] * temp[1] +
                  calibration->accel_a_matrix_inverse[2][2] * temp[2]));

    imu_msg->orientation = {0};

    imu_msg->orientation_covariance[0] = -1;
    imu_msg->linear_acceleration_covariance[0] = -1;
    imu_msg->angular_velocity_covariance[0] = -1;

    // IMU covariance

    imu_msg->angular_velocity_covariance[0] = calibration->angular_vel_covariance[0];
    imu_msg->angular_velocity_covariance[4] = calibration->angular_vel_covariance[1];
    imu_msg->angular_velocity_covariance[8] = calibration->angular_vel_covariance[2];

    imu_msg->linear_acceleration_covariance[0] = calibration->linear_accel_covariance[0];
    imu_msg->linear_acceleration_covariance[4] = calibration->linear_accel_covariance[1];
    imu_msg->linear_acceleration_covariance[8] = calibration->linear_accel_covariance[2];

    imu_msg->orientation_covariance[0] = calibration->orientation_covariance[0];
    imu_msg->orientation_covariance[4] = calibration->orientation_covariance[1];
    imu_msg->orientation_covariance[8] = calibration->orientation_covariance[2];
}
