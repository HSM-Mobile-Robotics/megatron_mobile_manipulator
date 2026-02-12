#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>

using namespace std::chrono_literals;

class robotModel : public rclcpp::Node {
public:
  robotModel()
      : Node("odom_publisher"), x_q_(0.0), y_q_(0.0), theta_q_(0.0), v_q_(0.0),
        w_q_(0.0) {

    subscription_cmd_vel_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&robotModel::cmd_vel_callback, this,
                      std::placeholders::_1));

    watchdog_timer_ = this->create_wall_timer(
        200ms, std::bind(&robotModel::watchdog_callback, this));

    publisher_odom_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ =
        this->create_wall_timer(100ms,
                                std::bind(&robotModel::update_odom, this));
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    watchdog_timer_->reset();
    // Get cmd_vel values
    v_cmd_ = msg->linear.x;
    w_cmd_ = msg->angular.z;
  }

  void watchdog_callback(){
    v_cmd_ = 0.0;
    w_cmd_ = 0.0;
  }
  void update_odom() {
    // Update state using model equations
    double dv_q = (-1.809 * v_q_) + (1.738 * v_cmd_);
    double dw_q = (-1.373 * w_q_) + ( 0.8597* w_cmd_);

    v_q_ += dv_q * 0.1; // Update velocity (assuming time step of 0.1s)
    w_q_ += dw_q * 0.1; // Update angular velocity

    // Compute position and orientation changes
    double dx_q = v_q_ * std::cos(theta_q_);
    double dy_q = v_q_ * std::sin(theta_q_);
    double dtheta_q = w_q_;

    // Update the robot's state
    x_q_ += dx_q * 0.1;
    y_q_ += dy_q * 0.1;
    theta_q_ += dtheta_q * 0.1;

    // Publish the odom message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_q_;
    odom_msg.pose.pose.position.y = y_q_;
    odom_msg.pose.pose.orientation.z = std::sin(theta_q_ / 2);
    odom_msg.pose.pose.orientation.w = std::cos(theta_q_ / 2);

    odom_msg.twist.twist.linear.x = v_q_;
    odom_msg.twist.twist.angular.z = w_q_;

    publisher_odom_->publish(odom_msg);

    // Publish transform (if using TF)
    publish_transform();
  }

  void publish_transform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = x_q_;
    t.transform.translation.y = y_q_;
    t.transform.rotation.z = std::sin(theta_q_ / 2);
    t.transform.rotation.w = std::cos(theta_q_ / 2);

    tf_broadcaster_->sendTransform(t);
  }

  // State variables
  double x_q_, y_q_, theta_q_;
  double v_q_, w_q_;
  double v_cmd_, w_cmd_;
  
  // ROS 2 Subscribers, Publishers, and Timers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      subscription_cmd_vel_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_, watchdog_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotModel>());
  rclcpp::shutdown();
  return 0;
}
