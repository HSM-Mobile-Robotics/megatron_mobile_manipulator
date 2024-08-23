//micro ros headers
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

//message type headers
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>

// teensy 4.1 specific encoder header
#include "QuadEncoder.h"

//Roboclaw motor controler headers
#include "RoboClaw.h"

//custom headers
#include "robot.h"
#include "config.h"

//defines
#define LED_PIN 13
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { return false; } \
  }
#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)


//connection states for micro controller and micro ros agent on the PC
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//Objects for ros node, publisher and subscriber.
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t vel_publisher;
rcl_subscription_t cmdvel_subscriber;
bool micro_ros_init_successful;

geometry_msgs__msg__Twist cmdvel_msg, vel_msg;


//timer variables
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
const uint64_t timer_timeout = 10;

//Motor driver object
RoboClaw roboclaw = RoboClaw(&Serial2, 10000);  //PIN 8 TX2 to roboclaw S1 Signal pin and PIN 7 RX2 to roboclaw S2 Signal pin

//Encoder objects
QuadEncoder leftEncoder(M1EncCh, M1EncPinA, M1EncPinB, 0);   // Encoder on channel 1 of 4 available,
                                                             // Phase A (pin0), PhaseB(pin1), Pullups Req(0)
QuadEncoder rightEncoder(M2EncCh, M2EncPinA, M2EncPinB, 0);  // Encoder on channel 1 of 4 available,
                                                             // Phase A (pin3), PhaseB(pin2), Pullups Req(0)

//Robot object
Robot robot(WHEEL_RADIUS, TRACK_WIDTH, MAXRPM);  // Wheel radius , track width, max rpm

//callback for the timer assiged to publisher
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    //Move the robot according to the input cmd_vel msg
    robot.setSpeed(&cmdvel_msg, prev_cmd_time, &vel_msg);
    /*
    robot.getRobotVelocity();

    vel_msg.linear.x = robot.velActual.left;
    vel_msg.linear.y = robot.velActual.right;
    */
    //publish all the topics
    publishTopics();
  }
}

void publishTopics() {

  //Updating the time stamps for the publishing msgs
  rcl_publish(&vel_publisher, &vel_msg, NULL);
}

// subscriber call back function
void cmdvel_sub_callback(const void *msgin) {
  prev_cmd_time = millis();
}

//If the connection is established the entities are created

bool create_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "ulrich_robot_node", "", &support));

  //create twist msg subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmdvel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/pid_cmd"));

  RCCHECK(rclc_publisher_init_best_effort(
    &vel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/velocity"));

  // create timer, it controlles the publish rate
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));  //2 handles one for timer and one for subscriber

  //adding timer to the executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  //adding subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &cmdvel_subscriber, &cmdvel_msg, &cmdvel_sub_callback, ON_NEW_DATA));

  // synchronize time with the agent
  syncTime();

  return true;
}

//If the connection is broke the entities are destroied
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&cmdvel_subscriber, &node);
  rcl_publisher_fini(&vel_publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void syncTime() {
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}

struct timespec getTime() {
  struct timespec tp = { 0 };
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;

  robot.EncodersInit(&leftEncoder, &rightEncoder, Encoder_Resolution);  //encoder variable address and resolution of the encoders
  robot.motorsInit(&roboclaw, BAUDRATE);                                //roboclaw object address and baudrate
  robot.M1pid.init(LEFT_k_p, LEFT_k_i, LEFT_k_d,-MAXCONTROLCOMMAND, MAXCONTROLCOMMAND, FILTERALPHA);
  robot.M2pid.init(RIGHT_k_p, RIGHT_k_i, RIGHT_k_d,-MAXCONTROLCOMMAND, MAXCONTROLCOMMAND, FILTERALPHA);
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}