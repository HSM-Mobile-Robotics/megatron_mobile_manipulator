/*
 
 MIT License
 Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab

 Authors:Venkata Prashanth Uppalapati <venkataprashanth.u@gamil.com>

*/


#ifndef _ROBOT_H_
#define _ROBOT_H_

// IMU headers
#include "imu.h"

// teensy 4.1 specific encoder header
#include "QuadEncoder.h"

// Roboclaw motor controller headers
#include "RoboClaw.h"

// PID

#include "PID.h"
#include "config.h"

// micro-ros headers
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>

// message type headers
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>


class Motor {
   public:
    QuadEncoder encoder;
    float encoderRes = 0;
    float velActual = 0, velReq;
    PID pid;
    Motor(uint8_t encoderCh, uint8_t phaseAPin, uint8_t phaseBPin, uint8_t pinPus,
          float encoderResolution)
        : encoder(encoderCh, phaseAPin, phaseBPin, pinPus), encoderRes(encoderResolution){};
};

class Robot {
   private:
    static const float wheelRadius, trackWidth, maxRPM, maxVel;  // robot kinematics variables

    double xPos = 0.0, yPos = 0.0, theta = 0.0;  // robot position variables

    unsigned long prevVelTime = 0.0, prevOdomTime = 0.0;  // time variables
    float velUpdateTime = 0.0;

    RoboClaw roboclaw;  // motor controller

   private:
    const void euler_to_quat(float roll, float pitch, float yaw, double *q);

   public:
    IMU imuICM;
    Motor motor1Left, motor2Right;

   public:
    Robot(nav_msgs__msg__Odometry *odom_msg, sensor_msgs__msg__Imu* imu_msg);
 
    void moveRobot(geometry_msgs__msg__Twist *step_cmd_msg, unsigned long prev_cmd_time, nav_msgs__msg__Odometry *odom_msg, geometry_msgs__msg__Twist *motor_msg);
    void updateOdometryData(nav_msgs__msg__Odometry *odom_msg);

    void getRobotVelocity();

};

#endif