#ifndef _ROBOT_H_
#define _ROBOT_H_

//IMU headers
#include "imu.h"

// teensy 4.1 specific encoder header
#include "QuadEncoder.h"

//Roboclaw motor controller headers
#include "RoboClaw.h"

//PID

#include "PID.h"

#include "config.h"


//micro-ros headers
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

//message type headers
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>

typedef struct {
  float left = 0.0, right = 0.0;
}Vel;

class Robot
{
private:
    float wheelRadius = 0.0, trackWidth = 0.0,
      maxRPM = 0.0, maxVel = 0.0, encoderRes = 0, velUpdateTime = 0.0;   
    double  xPos = 0.0, yPos = 0.0, theta = 0.0;
    unsigned long prevVelTime = 0.0, prevOdomTime = 0.0;
    uint32_t baudrate = 115200;

    RoboClaw *roboclaw;
    QuadEncoder *leftEncoder , *rightEncoder;
    nav_msgs__msg__Odometry *odom_msg;

private:
    const void euler_to_quat(float roll, float pitch, float yaw, double *q);
    
public:
    Vel velReq;
    Vel velActual;
    IMU imuICM;
    PID M1pid , M2pid;
public:
    Robot(float wheelRadius, float trackWidth, float maxRPM);
    void EncodersInit(QuadEncoder *leftencoder, QuadEncoder *rightencoder, float encoderRes);
    void motorsInit(RoboClaw *roboclaw, long baudrate);
    void OdometryInit(nav_msgs__msg__Odometry *odom_msg);

    void moveRobot(geometry_msgs__msg__Twist *cmdvel_msg, unsigned long prev_cmd_time);
    void updateOdometryData();

    void getRobotVelocity();

    //temp function for PID testing
    void setSpeed(geometry_msgs__msg__Twist *cmdvel_msg, unsigned long prev_cmd_time, geometry_msgs__msg__Twist *vel_msg);
};



#endif