// MIT License
// Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab
//
// Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MPCStepCmdPublisher : public rclcpp::Node
{

public:
    // Class constructor it run when the object for the class is created
    MPCStepCmdPublisher() : Node("mpc_step_cmd_publisher_node")
    {
        // Creating publisher object to handel the publishing operations
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/step_cmd", 10);

        // timer object to define the publishing rate, and the timer_callback function is bind to the timer
        timer_ = this->create_wall_timer(10ms, std::bind(&MPCStepCmdPublisher::timer_callback, this));

        this->declare_parameter<double>("step_value_linear_x", 10);
        this->declare_parameter<double>("step_value_angular_z", 10);

        step_value_linear_x_ = this->get_parameter("step_value_linear_x").as_double();
        step_value_angular_z_ = this->get_parameter("step_value_angular_z").as_double();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // publisher object declaration

    rclcpp::TimerBase::SharedPtr timer_; // timer object declaration

    void timer_callback(){
        auto message = geometry_msgs::msg::Twist();
        if(count_ < 200){
            message.linear.x = 0;
            message.angular.z = 0;
        }
        else if (count_ < 700){
            message.linear.x = step_value_linear_x_;
            message.angular.z = step_value_angular_z_;
        }
        else if (count_ < 1200){
            message.linear.x = 0;
            message.angular.z = 0;
        }
        else if (count_ < 1700){
            message.linear.x = -step_value_linear_x_;
            message.angular.z = -step_value_angular_z_;
        }
        else{
            message.linear.x = 0;
            message.angular.z = 0;
        }
        count_++;

        publisher_->publish(message);
    };

    int count_;
    double step_value_linear_x_, step_value_angular_z_;

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCStepCmdPublisher>());
    rclcpp::shutdown();
    return 0;
}
