#include "tracked_robot_control/track_drive_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace tracked_robot_control
{

TrackDriveController::TrackDriveController(const rclcpp::NodeOptions & options)
: rclcpp::Node("track_drive_controller", options)
{
  control_rate_ = declare_parameter<double>("control_rate", 50.0);
  max_linear_ = declare_parameter<double>("max_linear_velocity", 0.8);
  max_angular_ = declare_parameter<double>("max_angular_velocity", 1.5);
  max_linear_accel_ = declare_parameter<double>("max_linear_accel", 1.0);
  max_angular_accel_ = declare_parameter<double>("max_angular_accel", 3.0);
  teleop_timeout_ = declare_parameter<double>("teleop_timeout", 0.5);
  nav_timeout_ = declare_parameter<double>("nav_timeout", 0.5);
  angular_gain_ = declare_parameter<double>("angular_gain", 1.0);
  frame_id_ = declare_parameter<std::string>("frame_id", "base_footprint");
  const auto output_topic =
    declare_parameter<std::string>("output_topic", "diff_drive_controller/cmd_vel");
  const bool relay_odom = declare_parameter<bool>("relay_odom", true);

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 10);

  // manual (highest priority) and autonomous (Nav2) inputs
  teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&TrackDriveController::on_teleop, this, std::placeholders::_1));
  nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_nav", 10, std::bind(&TrackDriveController::on_nav, this, std::placeholders::_1));

  if (relay_odom) {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "diff_drive_controller/odom", 10,
      std::bind(&TrackDriveController::on_odom, this, std::placeholders::_1));
  }

  estop_srv_ = create_service<std_srvs::srv::SetBool>(
    "emergency_stop",
    std::bind(&TrackDriveController::on_estop, this, std::placeholders::_1, std::placeholders::_2));

  const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_, 1.0));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TrackDriveController::control_loop, this));

  RCLCPP_INFO(get_logger(), "track_drive_controller ready: /cmd_vel (manual) > /cmd_vel_nav (auto) -> %s",
    output_topic.c_str());
}

double TrackDriveController::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

double TrackDriveController::ramp(double current, double target, double max_delta)
{
  return current + clamp(target - current, -max_delta, max_delta);
}

void TrackDriveController::on_teleop(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  teleop_.linear = msg->linear.x;
  teleop_.angular = msg->angular.z;
  teleop_.stamp = now();
  teleop_.valid = true;
}

void TrackDriveController::on_nav(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  nav_.linear = msg->linear.x;
  nav_.angular = msg->angular.z;
  nav_.stamp = now();
  nav_.valid = true;
}

void TrackDriveController::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_pub_->publish(*msg);
}

void TrackDriveController::on_estop(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  estop_ = request->data;
  if (estop_) {
    current_linear_ = 0.0;
    current_angular_ = 0.0;
    teleop_.valid = false;
    nav_.valid = false;
  }
  response->success = true;
  response->message = estop_ ? "EMERGENCY STOP ENGAGED" : "Emergency stop released";
  RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
}

void TrackDriveController::control_loop()
{
  const auto t = now();
  const double dt = 1.0 / std::max(control_rate_, 1.0);

  double target_v = 0.0;
  double target_w = 0.0;
  std::string source = "none";

  if (!estop_) {
    if (teleop_.valid && (t - teleop_.stamp).seconds() < teleop_timeout_) {
      target_v = teleop_.linear;
      target_w = teleop_.angular;
      source = "teleop";
    } else if (nav_.valid && (t - nav_.stamp).seconds() < nav_timeout_) {
      target_v = nav_.linear;
      target_w = nav_.angular;
      source = "nav";
    }
  } else {
    source = "estop";
  }

  if (source != active_source_) {
    RCLCPP_INFO(get_logger(), "Active command source: %s", source.c_str());
    active_source_ = source;
  }

  target_v = clamp(target_v, -max_linear_, max_linear_);
  target_w = clamp(target_w * angular_gain_, -max_angular_, max_angular_);

  if (estop_) {
    current_linear_ = 0.0;
    current_angular_ = 0.0;
  } else {
    current_linear_ = ramp(current_linear_, target_v, max_linear_accel_ * dt);
    current_angular_ = ramp(current_angular_, target_w, max_angular_accel_ * dt);
  }

  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = t;
  out.header.frame_id = frame_id_;
  out.twist.linear.x = current_linear_;
  out.twist.angular.z = current_angular_;
  cmd_pub_->publish(out);
}

}  // namespace tracked_robot_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tracked_robot_control::TrackDriveController>());
  rclcpp::shutdown();
  return 0;
}
