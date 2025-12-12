// MIT License
// Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab
//
// Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class IMUGyroBiasCalibration: public rclcpp::Node
{
public:
    IMUGyroBiasCalibration(): Node("IMU_Gyro_bias_calibration")
    {
      publisher_ouster_ = this->create_publisher<std_msgs::msg::Float64>("calDegree_ouster", 10);
      publisher_icm_ = this->create_publisher<std_msgs::msg::Float64>("calDegree_icm", 10);

      subscription_ouster_ = this->create_subscription<sensor_msgs::msg::Imu>("ouster/imu", 10, std::bind(&IMUGyroBiasCalibration::ouster_topic_callback, this ,_1));
      subscription_icm_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data_raw", 10, std::bind(&IMUGyroBiasCalibration::icm_topic_callback, this, _1));

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_ouster_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_icm_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_ouster_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_icm_;

    double yaw_rad_ouster_ = 0.0, yad_rad_icm_ = 0.0;
    double prev_time_ouster_, prev_time_icm_;

    void ouster_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
      double current_time = this->now().nanoseconds();
      auto degree_msg = std_msgs::msg::Float64();
      if (prev_time_ouster_ > 0.0){
        double dt = current_time - prev_time_ouster_;

        double yaw_rate = msg->angular_velocity.z;
        yaw_rad_ouster_ += (yaw_rate*dt)/ 1e9;

        degree_msg.data = yaw_rad_ouster_ * 180.0 / M_PI;
        publisher_ouster_->publish(degree_msg);

      }
      prev_time_ouster_ = current_time;
    }

    void icm_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
      double current_time = this->now().nanoseconds();
      auto degree_msg = std_msgs::msg::Float64();
      if (prev_time_icm_ > 0.0){
        double dt = current_time - prev_time_icm_;

        double yaw_rate = msg->angular_velocity.z;
        yad_rad_icm_ += (yaw_rate*dt)/ 1e9;

        degree_msg.data = yad_rad_icm_ * 180.0 / M_PI;
        publisher_ouster_->publish(degree_msg);

      }
      prev_time_icm_ = current_time;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUGyroBiasCalibration>());
  rclcpp::shutdown();
  return 0;
}