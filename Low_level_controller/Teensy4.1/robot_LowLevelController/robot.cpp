#include "imxrt.h"
#include "robot.h"

const float Robot::wheelRadius = WHEEL_RADIUS;
const float Robot::trackWidth = TRACK_WIDTH;
const float Robot::maxRPM = MAX_RPM;
const float Robot::maxVel = (2 * 3.141592653 * Robot::wheelRadius * Robot::maxRPM) / 60.0;

Robot::Robot(nav_msgs__msg__Odometry *odom_msg, sensor_msgs__msg__Imu *imu_msg, sensor_msgs__msg__MagneticField* mag_msg)
    : roboclaw(&MOTOR_CONTROLLER_SERIAL_PORT, 10000),
      imuICM(CS_PIN, SPI_PORT, SPI_FREQ,&IMU_CALIB),
      motor1Left(MOTOR1_ENCODER_CHANNEL, MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B, 0,
                 ENCODER_RESOLUTION),
      motor2Right(MOTOR2_ENCODER_CHANNEL, MOTOR2_ENCODER_PIN_A, MOTOR2_ENCODER_PIN_B, 0,
                  ENCODER_RESOLUTION) {
    roboclaw.begin(MOTOR_CONTROLLER_BAUDRATE);
    motor1Left.encoder.setInitConfig();
    motor1Left.encoder.init();
    motor2Right.encoder.setInitConfig();
    motor2Right.encoder.init();
    motor1Left.pid.init(MOTOR1_k_p, MOTOR1_k_i, MOTOR1_k_d, -MAX_CONTROL_COMMAND,
                        MAX_CONTROL_COMMAND, FILTER_ALPHA);
    motor2Right.pid.init(MOTOR2_k_p, MOTOR2_k_i, MOTOR2_k_d, -MAX_CONTROL_COMMAND,
                         MAX_CONTROL_COMMAND, FILTER_ALPHA);

    odom_msg->header.frame_id =
        micro_ros_string_utilities_set(odom_msg->header.frame_id, ODOM_FRAME);
    odom_msg->child_frame_id = micro_ros_string_utilities_set(odom_msg->child_frame_id, BASE_FRAME);
    imu_msg->header.frame_id = micro_ros_string_utilities_set(imu_msg->header.frame_id, IMU_FRAME);
    mag_msg->header.frame_id = micro_ros_string_utilities_set(mag_msg->header.frame_id, MAG_FRAME);
}

// Updating actual velocity from the encoder counts
void Robot::getRobotVelocity() {
    unsigned long currentTime = micros();

    float leftEncCounts = motor1Left.encoder.read();
    float rightEncCounts = motor2Right.encoder.read();

    motor1Left.encoder.write(0);
    motor2Right.encoder.write(0);

    unsigned long dt = currentTime - prevVelTime;
    velUpdateTime = (float)dt / 1000000;
    prevVelTime = currentTime;

    motor1Left.velActual =
        (((float)leftEncCounts / motor1Left.encoderRes) * wheelRadius * PI * 2) / velUpdateTime;
    motor2Right.velActual =
        (((float)rightEncCounts / motor2Right.encoderRes) * wheelRadius * PI * 2) / velUpdateTime;
}

// Updating odometry data from the actual velocity
void Robot::updateOdometryData(nav_msgs__msg__Odometry *odom_msg) {
    unsigned long currentTime = micros();
    unsigned long dt = currentTime - prevOdomTime;
    float updateTime = (float)dt / 1000000;
    prevOdomTime = currentTime;

    // calculating the change in linear velocity and angular velocity
    float dv = ((motor2Right.velActual + motor1Left.velActual) * (updateTime)) / 2;
    float dTheta = ((motor2Right.velActual - motor1Left.velActual) * (updateTime)) / trackWidth;

    // Calculating the change in postion in x and y coordinates
    float dx = cos(dTheta) * dv;
    float dy = sin(dTheta) * dv;

    // Calculating the cumulative position change
    xPos += (cos(theta) * dx - sin(theta) * dy);
    yPos += (cos(theta) * dx + sin(theta) * dy);
    theta += dTheta;

    // Reseting the angle after a complete rotation
    if (theta >= TWO_PI) theta -= TWO_PI;
    if (theta <= -TWO_PI) theta += TWO_PI;

    // changing the euler data to quaternion
    double q[4];
    euler_to_quat(0, 0, theta, q);

    // robot position in x, y, z;
    odom_msg->pose.pose.position.x = xPos;
    odom_msg->pose.pose.position.y = yPos;
    odom_msg->pose.pose.position.z = 0.0;

    // robot's heading in quaternion
    odom_msg->pose.pose.orientation.x = q[1];
    odom_msg->pose.pose.orientation.y = q[2];
    odom_msg->pose.pose.orientation.z = q[3];
    odom_msg->pose.pose.orientation.w = q[0];

    // Pose covariance
    odom_msg->pose.covariance[0] = POSE_COVARIANCE[0];
    odom_msg->pose.covariance[7] = POSE_COVARIANCE[1];
    odom_msg->pose.covariance[14] = POSE_COVARIANCE[2];
    odom_msg->pose.covariance[21] = POSE_COVARIANCE[3];
    odom_msg->pose.covariance[28] = POSE_COVARIANCE[4];
    odom_msg->pose.covariance[35] = POSE_COVARIANCE[5];

    // linear speed from encoders
    odom_msg->twist.twist.linear.x = (double)((motor2Right.velActual + motor1Left.velActual) / 2);

    // angular speed from encoders
    odom_msg->twist.twist.angular.z =
        (double)((motor2Right.velActual - motor1Left.velActual) / trackWidth);

    // twist covariance
    odom_msg->twist.covariance[0] = TWIST_COVARIANCE[0];
    odom_msg->twist.covariance[7] = TWIST_COVARIANCE[1];
    odom_msg->twist.covariance[14] = TWIST_COVARIANCE[2];
    odom_msg->twist.covariance[21] = TWIST_COVARIANCE[3];
    odom_msg->twist.covariance[28] = TWIST_COVARIANCE[4];
    odom_msg->twist.covariance[35] = TWIST_COVARIANCE[5];
}

// Move the robot according the cmd_velocity
void Robot::moveRobot(geometry_msgs__msg__Twist *cmdvel_msg, unsigned long prev_cmd_time,
                      nav_msgs__msg__Odometry *odom_msg) {
    // braking if there's no command received after 100ms
    if ((millis() - prev_cmd_time) >= 100) {
        motor1Left.velReq = 0.0;
        motor2Right.velReq = 0.0;
    } else {
        motor2Right.velReq = (cmdvel_msg->linear.x + (cmdvel_msg->angular.z * (trackWidth / 2)));
        motor1Left.velReq = (cmdvel_msg->linear.x - (cmdvel_msg->angular.z * (trackWidth / 2)));
    }

    getRobotVelocity();

    bool coastflag;

    if ((motor1Left.velReq == 0 || motor2Right.velReq == 0) &&
        (abs(motor1Left.velActual) < 0.25 || abs(motor2Right.velActual) < 0.25)) {
        coastflag = 1;
    } else {
        coastflag = 0;
    }

    int controlValueLeft =
        motor1Left.pid.update(motor1Left.velReq, motor1Left.velActual, velUpdateTime, coastflag);
    int controlValueRight =
        motor2Right.pid.update(motor2Right.velReq, motor2Right.velActual, velUpdateTime, coastflag);

    if (controlValueRight >= 0)
        roboclaw.ForwardM2(0x80, controlValueRight);
    else
        roboclaw.BackwardM2(0x80, -controlValueRight);

    if (controlValueLeft >= 0)
        roboclaw.ForwardM1(0x80, controlValueLeft);
    else
        roboclaw.BackwardM1(0x80, -controlValueLeft);

    updateOdometryData(odom_msg);
}

// roll, yaw and pitch are in rad/sec

const void Robot::euler_to_quat(float roll, float pitch, float yaw, double *q) {
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}
