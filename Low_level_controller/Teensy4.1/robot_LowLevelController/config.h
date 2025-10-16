#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <array>

#include "imu.h"
/* ROBOT

    Front
MOTOR1  MOTOR2
    Back

*/

/* Robot Kinematics */
#define WHEEL_RADIUS 0.15  // wheel radius in meters
#define TRACK_WIDTH 0.525  // distance between left wheel to right wheel
#define MAX_RPM 150        // Max rpm for the wheels

/* Encoder parameters*/
#define MOTOR1_ENCODER_CHANNEL 1  // Quad encoder channel for motor1
#define MOTOR2_ENCODER_CHANNEL 2  // Quad encoder channel for motor2

#define MOTOR1_ENCODER_PIN_A \
    1  // encoder PhaseA Pin  for motor1,
       // Here pins are assigned reversed to that of pcb for the motor direction change
#define MOTOR1_ENCODER_PIN_B \
    0  // encoder PhaseB Pin  for motor1
       // Here pins are assigned reversed to that of pcb for the motor direction change

#define MOTOR2_ENCODER_PIN_A 2  // encoder PhaseA Pin  for motor2
#define MOTOR2_ENCODER_PIN_B 3  // encoder PhaseB Pin  for motor2

#define ENCODER_RESOLUTION 20480  // Encoder resolution

/* Motor controller parameters*/
#define MOTOR_CONTROLLER_BAUDRATE 460800
#define MOTOR_CONTROLLER_SERIAL_PORT Serial2
#define MAX_CONTROL_COMMAND 126  // Max packet value to the roboclaw controller
// #define MOTOR_CONTROLLER_ADDRESS 0x80

/* exponential moving average (fist order lowpass filter for PID) */
#define FILTER_ALPHA 0.90

/* Motor1 PID gains */
#define MOTOR1_k_p 15.61
#define MOTOR1_k_i 108.2
#define MOTOR1_k_d 0

/* Motor2 PID gains */
#define MOTOR2_k_p 15.61
#define MOTOR2_k_i 108.2
#define MOTOR2_k_d 0

// covariance for odometry message

static const float POSE_COVARIANCE[6] = {0.0001, 0.0001, 0, 0, 0, 0.0001};
static const float TWIST_COVARIANCE[6] = {0.00001, 0.00001, 0, 0, 0, 0.00001};

/*IMU*/
#define SPI_PORT SPI      // Your desired SPI port.       Used only when "USE_SPI" is defined
#define SPI_FREQ 4000000  // You can override the default SPI frequency
#define CS_PIN 10         // Which pin you connect CS to. Used only when "USE_SPI" is defined

/*Calibration data*/

// The offset and matrix are obtained calibration code from
// https://github.com/jremington/ICM_20948-AHRS.git

// GYRO

constexpr float GYRO_SCALING_FACTOR = 0.0002664625;  // scaling factor to change dps to rad/sec
constexpr float GYRO_OFFSET[3] = {-51.5, -7.9, 5.1};

// Accel

constexpr float ACCEL_SCALING_FACTOR = 0.00059855;  // scaling factor to change mg to m/sec²
constexpr float ACCEL_BIAS[3] = {39.41, -415.64, 408.7};  // Acceleration combined bias vector
// A is the matrix combining scale factors, misalignments and soft-iron effects, inverted for
// calibrated value calculation equation
constexpr float ACCEL_A_MATRIX_INVERSE[3][3] = {
{ 0.0609 , 0.00016 , -2e-05 },
{ 0.00016 , 0.06125 , 0.00241 },
{ -2e-05 , 0.00241 , 0.06117 }};

// Mag scale
constexpr float MAG_SCALING_FACTOR = 0.001;
constexpr float MAG_BIAS[3] = {-106.38, 393.59, -172.43};  // mag combined bias vector

// A is the matrix combining scale factors, misalignments and soft-iron effects, inverted for
// calibrated value calculation equation
constexpr float MAG_A_MATRIX_INVERSE[3][3] = {{2.16182, 0.15981, -0.62735},
                {0.15981, 2.31317, 0.05849}, {
    -0.62735, 0.05849, 2.35421}};
// covariance for IMU message

constexpr float LINEAR_ACCEL_COVARIANCE[3] = {0.00001, 0.00001, 0.00001};
constexpr float ANGULAR_VEL_COVARIANCE[3] = {0.00001, 0.00001, 0.00001};
constexpr float ORIENTATION_COVARIANCE[3] = {0.00001, 0.00001, 0.00001};
constexpr float MAGNETIC_FIELD_COVARIANCE[3] = {0.00001, 0.00001, 0.00001};

constexpr ImuCalibration IMU_CALIB = {GYRO_SCALING_FACTOR,
                                      GYRO_OFFSET,
                                      ACCEL_SCALING_FACTOR,
                                      ACCEL_BIAS,
                                      ACCEL_A_MATRIX_INVERSE,
                                      MAG_SCALING_FACTOR,
                                      MAG_BIAS,
                                      MAG_A_MATRIX_INVERSE,
                                      LINEAR_ACCEL_COVARIANCE,
                                      ANGULAR_VEL_COVARIANCE,
                                      ORIENTATION_COVARIANCE,
                                      MAGNETIC_FIELD_COVARIANCE};

// ROS TOPIC LIST AND CONFIGS

#define NODE_NAME "ulrich_robot_node"
#define CMD_VEL_TOPIC "/cmd_vel"
#define ODOM_TOPIC "/odom"
#define IMU_TOPIC "/imu/data_raw"
#define MAG_TOPIC "/imu/mag"

#define BASE_FRAME "base_link"
#define ODOM_FRAME "odom"
#define IMU_FRAME "imu_link"
#define MAG_FRAME "mag_link"

#define PUBLISH_RATE 10  //

#endif