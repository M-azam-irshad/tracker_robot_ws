#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace tracked_robot_control
{

/// Command arbitration + safety layer in front of diff_drive_controller.
class TrackDriveController : public rclcpp::Node
{
public:
  explicit TrackDriveController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Command
  {
    double linear{0.0};
    double angular{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void on_teleop(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_nav(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_estop(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void control_loop();

  static double clamp(double v, double lo, double hi);
  static double ramp(double current, double target, double max_delta);

  // parameters
  double control_rate_{50.0};
  double max_linear_{0.8};
  double max_angular_{1.5};
  double max_linear_accel_{1.0};
  double max_angular_accel_{3.0};
  double teleop_timeout_{0.5};
  double nav_timeout_{0.5};
  double angular_gain_{1.0};
  std::string frame_id_{"base_footprint"};

  // state
  Command teleop_;
  Command nav_;
  double current_linear_{0.0};
  double current_angular_{0.0};
  bool estop_{false};
  std::string active_source_{"none"};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tracked_robot_control
