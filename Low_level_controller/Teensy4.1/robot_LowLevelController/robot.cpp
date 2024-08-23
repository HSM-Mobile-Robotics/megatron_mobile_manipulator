#include "imxrt.h"
#include "robot.h"

#define address 0x80

//The robot class object constructor 
Robot::Robot(float wheelRadius, float trackWidth, float maxRPM){
    this->wheelRadius = wheelRadius;
    this->trackWidth = trackWidth;
    this->maxRPM = maxRPM;
    this->maxVel = (maxRPM*2*PI*wheelRadius)/60;
}

//Roboclaw motor driver initialization
void Robot::motorsInit(RoboClaw *roboclaw, long baudrate)
{
    this->roboclaw = roboclaw;
    this->baudrate = baudrate;
    this->roboclaw->begin(baudrate);
}

//Encoder initialization function
void Robot::EncodersInit(QuadEncoder *leftEncoder, QuadEncoder *rightEncoder , float encoderRes){
    this->encoderRes = encoderRes;
    this->leftEncoder = leftEncoder;
    this->rightEncoder = rightEncoder;
    leftEncoder->setInitConfig();
    leftEncoder->init();
    rightEncoder->setInitConfig();
    rightEncoder->init();  
}

//Odometry msg initialization
void Robot::OdometryInit(nav_msgs__msg__Odometry *odom_msg){
    this->odom_msg = odom_msg;
    odom_msg->header.frame_id = micro_ros_string_utilities_set(odom_msg->header.frame_id, "odom");
    odom_msg->child_frame_id = micro_ros_string_utilities_set(odom_msg->child_frame_id, "base_link");
}

//Updating actual velocity from the encoder counts 
void Robot::getRobotVelocity(){

  unsigned long currentTime = micros();

  float leftEncCounts = leftEncoder->read();
  float rightEncCounts = rightEncoder->read();

  leftEncoder->write(0);
  rightEncoder->write(0);

  unsigned long dt = currentTime - prevVelTime;
  velUpdateTime = (float)dt/1000000;
  prevVelTime = currentTime;


  if(abs(leftEncCounts)< 25 && abs(leftEncCounts)< 25){
    velActual.left = 0;
    velActual.right = 0;
  }
  else {
    velActual.left = (((float)leftEncCounts/encoderRes)*wheelRadius*PI*2)/velUpdateTime;
    velActual.right = (((float)rightEncCounts/encoderRes)*wheelRadius*PI*2)/velUpdateTime;
  }
 
}

//Updating odometry data from the actual velocity
void Robot::updateOdometryData(){

  unsigned long currentTime = micros();
  unsigned long dt = currentTime - prevOdomTime;
  float updateTime = (float)dt/1000000;
  prevOdomTime = currentTime;

  //calculating the change in linear velocity and angular velocity
  float dv = ((velActual.right+velActual.left)*(updateTime))/2;
  float dTheta = ((velActual.right-velActual.left)*(updateTime))/trackWidth;

  //Calculating the change in postion in x and y coordinates  
  float dx= cos(dTheta)*dv;
  float dy= sin(dTheta)*dv;

  //Calculating the cumulative position change
  xPos += (cos(theta)*dx - sin(theta)*dy);
  yPos += (cos(theta)*dx + sin(theta)*dy);
  theta += dTheta;

  //Reseting the angle after a complete rotation  
  if(theta >= TWO_PI) theta -= TWO_PI;
  if(theta <= -TWO_PI) theta += TWO_PI;

  //changing the euler data to quaternion
  double q[4];
  euler_to_quat(0,0,theta,q);

  //robot position in x, y, z;
  odom_msg->pose.pose.position.x = xPos;
  odom_msg->pose.pose.position.y = yPos;
  odom_msg->pose.pose.position.z = 0.0;

  //robot's heading in quaternion
  odom_msg->pose.pose.orientation.x = q[1];
  odom_msg->pose.pose.orientation.y = q[2];
  odom_msg->pose.pose.orientation.z = q[3];
  odom_msg->pose.pose.orientation.w = q[0];

  //linear speed from encoders
  odom_msg->twist.twist.linear.x = (double)((velActual.right+velActual.left)/2);

  //angular speed from encoders
  odom_msg->twist.twist.angular.z = (double)((velActual.right-velActual.left)/trackWidth);
}

//Move the robot according the cmd_velocity
void Robot::moveRobot(geometry_msgs__msg__Twist *cmdvel_msg, unsigned long prev_cmd_time){
    // braking if there's no command received after 200ms
    if((millis()-prev_cmd_time)>=200){
      velReq.left = 0.0;
      velReq.right = 0.0;
    }
    else{
      velReq.right = (cmdvel_msg->linear.x + (cmdvel_msg->angular.z*(trackWidth/2)));
      velReq.left = (cmdvel_msg->linear.x - (cmdvel_msg->angular.z*(trackWidth/2)));
    }
    
    getRobotVelocity();

    int controlValueLeft =  M1pid.update(velReq.left, velActual.left, velUpdateTime);
    int controlValueRight = M2pid.update(velReq.right, velActual.right, velUpdateTime);
  
    if(controlValueRight >= 0 )
        roboclaw->ForwardM2(address, controlValueRight);
    else
        roboclaw->BackwardM2(address, -controlValueRight);
    
    if(controlValueLeft >= 0)
        roboclaw->ForwardM1(address, controlValueLeft);
    else
        roboclaw->BackwardM1(address, -controlValueLeft);

    updateOdometryData();
}


//temp function for PID testing
void Robot::setSpeed(geometry_msgs__msg__Twist *cmdvelMsg, unsigned long prevCmdTime, geometry_msgs__msg__Twist *velMsg){
    // braking if there's no command received after 200ms
    if((millis()-prevCmdTime)>=200){
      velReq.left = 0.0;
      velReq.right = 0.0;
    }
    else{
      /*velReq.left =  cmdvelMsg->linear.x;
      velReq.right = cmdvelMsg->linear.y;   

      */
      velReq.left = (cmdvelMsg->linear.x - (cmdvelMsg->angular.z*(trackWidth/2)));
      velReq.right = (cmdvelMsg->linear.x + (cmdvelMsg->angular.z*(trackWidth/2)));
       
    }
    
    getRobotVelocity();

    /*
    controlValue.left = velReq.left;
    controlValue.right =  velReq.right;
    */
    int controlValueLeft =  M1pid.update(velReq.left, velActual.left, velUpdateTime);
    int controlValueRight = M2pid.update(velReq.right, velActual.right, velUpdateTime);
    
    if(controlValueRight >= 0 )
       roboclaw->ForwardM2(address, controlValueRight);
    else
        roboclaw->BackwardM2(address, -controlValueRight);
    if(controlValueLeft >= 0)
        roboclaw->ForwardM1(address, controlValueLeft);
    else
        roboclaw->BackwardM1(address, -controlValueLeft);
   
    velMsg->linear.x = velReq.left;
    velMsg->linear.y = velActual.left;
    velMsg->linear.z = controlValueLeft;
    velMsg->angular.x = velReq.right;
    velMsg->angular.y = velActual.right;
    velMsg->angular.z = controlValueRight;

}

//roll, yaw and pitch are in rad/sec

const void Robot::euler_to_quat(float roll, float pitch, float yaw, double *q) 
{
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