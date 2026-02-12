/*

MIT License
Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab

Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

*/

// micro ros headers
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <stdio.h>

// message type headers
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

// custom headers
#include "config.h"
#include "robot.h"

// defines
#define LED_PIN 13
#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      return false;                                                            \
    }                                                                          \
  }
#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static volatile int64_t init = -1;                                         \
    if (init == -1) {                                                          \
      init = uxr_millis();                                                     \
    }                                                                          \
    if (uxr_millis() - init > MS) {                                            \
      X;                                                                       \
      init = uxr_millis();                                                     \
    }                                                                          \
  } while (0)

// connection states for micro controller and micro ros agent on the PC
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Objects for ros node
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// Objects for publisher and subscriber
rcl_publisher_t odom_publisher, imu_publisher;
rcl_subscription_t cmd_vel_subscriber;
bool micro_ros_init_successful;

// Ros msg variables
geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;

// timer variables
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

// Robot object
Robot robot(&odom_msg, &imu_msg);

// callback for the timer assigned to publisher
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Move the robot according to the input cmd_vel msg
    robot.moveRobot(&cmd_vel_msg, prev_cmd_time, &odom_msg);

    // publish all the topics
    publishTopics();
  }
}

void publishTopics() {
  // Updating the time stamps for the publishing msgs
  struct timespec time_stamp = getTime();

  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  imu_msg.header.stamp.sec = time_stamp.tv_sec;
  imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  robot.imuICM.updateData(&imu_msg);

  rcl_publish(&odom_publisher, &odom_msg, NULL);

  rcl_publish(&imu_publisher, &imu_msg, NULL);
}

// subscriber call back function
void cmdvel_sub_callback(const void *msgin) { prev_cmd_time = millis(); }

// If the connection is established the entities are created

bool create_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "ulrich_robot_node", "", &support));

  // create twist msg subscriber
  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_TOPIC));

  RCCHECK(rclc_publisher_init_default(
      &odom_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ODOM_TOPIC));

  RCCHECK(rclc_publisher_init_default(
      &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      IMU_TOPIC));

  // create timer, it controls the publish rate
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10),
                                  timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(
      &executor, &support.context, 2,
      &allocator)); // 2 handles one for timer and one for subscriber

  // adding timer to the executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // adding subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber,
                                         &cmd_vel_msg, &cmdvel_sub_callback,
                                         ON_NEW_DATA));

  // synchronize time with the agent
  syncTime();

  return true;
}

// If the connection is broke the entities are destroyed
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&cmd_vel_subscriber, &node);
  rcl_publisher_fini(&odom_publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);

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
  struct timespec tp = {0};
  // add time difference between uC time and ROS time to
  // synchronize time with ROS0
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  state = WAITING_AGENT;

  robot.imuICM.imuInit(CS_PIN, SPI_PORT, SPI_FREQ);
}

void loop() {
  switch (state) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_AVAILABLE
                                        : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_CONNECTED
                                        : AGENT_DISCONNECTED;);
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
