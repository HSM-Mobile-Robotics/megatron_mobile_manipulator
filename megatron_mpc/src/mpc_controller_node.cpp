// MIT License
// Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab
//
// Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

// Inspired from lifecycle nodes demo and crazyflie Mpc implementation

// Standard
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

// ROS
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"

#include "lifecycle_msgs/msg/transition.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

// acados
#include "acados/utils/math.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"

// Megatron acados specific
#include "acados_solver_megatron.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX MEGATRON_NX
#define NP MEGATRON_NP
#define NU MEGATRON_NU
#define NBX0 MEGATRON_NBX0
#define N MEGATRON_N
#define NY MEGATRON_NY

using namespace std::chrono_literals;
using std::placeholders::_1;

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NMPC : public LifecycleNode {

public:
  explicit NMPC(const std::string &nodeName, bool intraProcessComms = false)
      : LifecycleNode(nodeName, rclcpp::NodeOptions().use_intra_process_comms(
                                    intraProcessComms)) {
    RCLCPP_INFO(get_logger(), "NMPC Lifecycle node created.");
  }
  // -------------------- LIFECYCLE CALLBACKS --------------------

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) {

    RCLCPP_INFO(get_logger(), "Configuring NMPC node...");

    publisher_cmdVel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);

    subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 5, std::bind(&NMPC::odom_topic_callback, this, _1));

    timer_mpc_ = this->create_wall_timer(
        100ms, std::bind(&NMPC::solve_mpc_callback, this));

    timer_mpc_->cancel(); // Pausing without destroying the object

    try {
      acados_capsule_ = megatron_acados_create_capsule();

      status_ = megatron_acados_create(acados_capsule_);
      if (status_ != 0) {
        throw std::runtime_error("megatron_acados_create() returned status " +
                                 std::to_string(status_));
      }
      nlp_config = megatron_acados_get_nlp_config(acados_capsule_);
      nlp_dims = megatron_acados_get_nlp_dims(acados_capsule_);
      nlp_in = megatron_acados_get_nlp_in(acados_capsule_);
      nlp_out = megatron_acados_get_nlp_out(acados_capsule_);
      nlp_solver = megatron_acados_get_nlp_solver(acados_capsule_);

      RCLCPP_INFO(this->get_logger(), "ACADOS OCP successfully created!");
    } catch (const std::runtime_error &e) {
      RCLCPP_FATAL(this->get_logger(), "ACADOS init failed: %s", e.what());
      return CallbackReturn::FAILURE;
    }

    std::string pkg_path =
        ament_index_cpp::get_package_share_directory("megatron_mpc");
    std::string filename_ = pkg_path + "/data/reference_traj_kinematics.csv";

    // Load reference
    yref_full_ = get_reference(filename_);

    RCLCPP_INFO(this->get_logger(), "Reference matrix loaded (%ld x %ld):",
                static_cast<long>(yref_full_.rows()),
                static_cast<long>(yref_full_.cols()));

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) {

    publisher_cmdVel_->on_activate(); // Activate to publish messages

    timer_mpc_->reset(); // Reset the time to start solving the mpc

    horizon_step_ = 0; // initaliasing  the moving horizon steps to zero for mpc
                       // to start new

    RCLCPP_INFO(get_logger(), "Activating NMPC node...");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Deactivating NMPC node...");

    // Pausing without destroying the object
    timer_mpc_->cancel();
    publisher_cmdVel_->on_deactivate();

    subscription_odometry_
        .reset(); // Reset to release resources and stop receiving msgs

    RCLCPP_INFO(this->get_logger(), "Timer stopped during deactivation.");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Cleaning up NMPC node...");

    // Reset to release resources
    timer_mpc_.reset();
    publisher_cmdVel_.reset();
    subscription_odometry_.reset();

    status_ = megatron_acados_free(acados_capsule_);
    if (status_ != 0) {
      int status = megatron_acados_free(acados_capsule_);
      if (status != 0)
        RCLCPP_WARN(get_logger(), "megatron_acados_free() returned status %d",
                    status);
      status = megatron_acados_free_capsule(acados_capsule_);
      if (status != 0)
        RCLCPP_WARN(get_logger(),
                    "megatron_acados_free_capsule() returned status %d",
                    status);
    }

    acados_capsule_ = nullptr;

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) {

    // Reset to release resources
    timer_mpc_.reset();
    publisher_cmdVel_.reset();
    subscription_odometry_.reset();

    RCLCPP_INFO(get_logger(), "NMPC node shutting down...");

    return CallbackReturn::SUCCESS;
  }

private:
  // -------------------- CALLBACKS --------------------

  void odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_odom_ = *msg; // Update local state
  }

  void solve_mpc_callback() {

    // -------------------- Get current state --------------------

    auto current_state = nav_msgs::msg::Odometry();
    {                                                 // <--- SCOPE STARTS
      std::lock_guard<std::mutex> lock(state_mutex_); // Mutex LOCKED
      current_state = latest_odom_;
    } // <--- SCOPE ENDS: Mutex is AUTOMATICALLY UNLOCKED here

    // update the current states to the solver (Feedback states)
    x_init_[0] = current_state.pose.pose.position.x;
    x_init_[1] = current_state.pose.pose.position.y;
    x_init_[2] = this->get_yaw_from_odom(current_state);
    x_init_[3] = current_state.twist.twist.linear.x;
    x_init_[4] = current_state.twist.twist.angular.z;

    auto cmd_vel_result = geometry_msgs::msg::Twist();

    cmd_vel_result = solve_nmpc(); // Solve MPC for control input

    publisher_cmdVel_->publish(cmd_vel_result);
    RCLCPP_INFO(this->get_logger(), "odom state %f and cmd vel %f",
                current_state.pose.pose.position.x, cmd_vel_result.linear.x);

    horizon_step_++; // incrementing the step for moving horizon
  };

  geometry_msgs::msg::Twist solve_nmpc() {

    // updating initial states to solver from the feedback received in odom msg

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0,
                                  "lbx", x_init_);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0,
                                  "ubx", x_init_);

    // Loop to update the yref for the next 50 steps from the horizon step.
    for (int j = 0; j < N; j++) {
      int global_idx = horizon_step_ + j;
      if (global_idx < yref_full_.rows()) {
        Eigen::VectorXd stage_ref = yref_full_.row(global_idx);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, j, "yref",
                               stage_ref.data());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Index out of bounds");
      }
    }
    // Last Stage (Terminal)
    int terminal_idx = horizon_step_ + N;
    if (terminal_idx < yref_full_.rows()) {
      Eigen::VectorXd yref_N = yref_full_.row(terminal_idx).head(NX);
      ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref",
                             yref_N.data());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Index out of bounds on last stage");
    }

    // Solve the ocp problem
    status_ = megatron_acados_solve(acados_capsule_);
    ocp_nlp_get(nlp_solver, "time_tot", &elapsed_time_);
    min_time_ = MIN(elapsed_time_, min_time_);

    if (status_ != ACADOS_SUCCESS) {
      RCLCPP_FATAL(this->get_logger(),
                   "megatron_acados_solve() failed with status %d", status_);
    }

    // get control input solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u_out_);

    // Solver performance
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf",
                    &kkt_norm_inf_);
    ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter_);

    RCLCPP_INFO(this->get_logger(),
                "\nSolver info: SQP iterations %2d  minimum time to solve %f "
                "[ms]  KKT %f",
                sqp_iter_, min_time_ * 1000.0, kkt_norm_inf_);

    // Updating the cmd_vel msg from the solution
    auto control_out = geometry_msgs::msg::Twist();

    control_out.linear.x = u_out_[0];
    control_out.angular.z = u_out_[1];

    return control_out;
  }

  // Get the reference from the .csv file
  Eigen::MatrixXd get_reference(const std::string &filename) {

    std::ifstream file(filename);

    if (!file.is_open()) {
      std::cerr << "Error opening file: " << filename << std::endl;
      return Eigen::MatrixXd(); // return empty matrix on error
    }

    std::vector<std::vector<double>> data;
    std::string line;

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::vector<double> row;
      std::string value;

      while (std::getline(ss, value, ',')) {
        row.push_back(std::stod(value));
      }

      data.push_back(row);
    }

    file.close();

    const int rows = static_cast<int>(data.size());
    const int cols = data.empty() ? 0 : static_cast<int>(data[0].size());

    Eigen::MatrixXd yref_full(rows, cols);
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        yref_full(i, j) = data[i][j];
      }
    }

    return yref_full;
  }

  // Fuction changes the orientation quaternion from subscribed odom msg to yaw
  // for MPC
  double get_yaw_from_odom(const nav_msgs::msg::Odometry &odom_msg) {
    tf2::Quaternion q(
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw; // radians
  }

  // -------------------- MEMBERS --------------------

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
      publisher_cmdVel_; // publisher object declaration

  rclcpp::TimerBase::SharedPtr timer_mpc_; // timer object declaration

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      subscription_odometry_;

  // Shared state protection
  // std::mutex to ensure the MPC loop doesn't read the state while the
  // Odometry callback is halfway through updating it.

  std::mutex state_mutex_;
  nav_msgs::msg::Odometry latest_odom_;

  // acados variables
  megatron_solver_capsule *acados_capsule_;
  int status_;

  ocp_nlp_config *nlp_config;
  ocp_nlp_dims *nlp_dims;
  ocp_nlp_in *nlp_in;
  ocp_nlp_out *nlp_out;
  ocp_nlp_solver *nlp_solver;
  void *nlp_opts;

  // initial states
  double x_init_[NX];

  // Control input
  double u_out_[NU];

  // solver evaluation
  double min_time_ = 1e12;
  double kkt_norm_inf_;
  double elapsed_time_;
  int sqp_iter_;

  int horizon_step_; // variable used to move the horizon after every timestep

  // reference trajectory acados require 7 values, 5 states and 2 control inputs
  // Size of the Matrix is Total {[(run time* discrete steps per second) +
  // (horizon steps +1) ]* yref states} {[( 10s*10) +(50 +1)]*7}
  Eigen::MatrixXd yref_full_;
};

int main(int argc, char *argv[]) {

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  // The lifecycle node is not spinned continuously to control the node state
  // spinned used threads
  rclcpp::executors::SingleThreadedExecutor exe;

  auto nmpc_node = std::make_shared<NMPC>("nmpc_node");

  exe.add_node(nmpc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
