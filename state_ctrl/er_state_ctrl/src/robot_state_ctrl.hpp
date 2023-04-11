// #pragma once
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state_ctrl.hpp"
#include "std_msgs/msg/string.hpp"

// ros class
class RobotStateCtrl : public rclcpp::Node {
 public:
  RobotStateCtrl();
  ~RobotStateCtrl() {}
  // publish mode as topic
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  // subscribe mode changer as topic
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_toggle_sub_;
  // publish state as topic
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  // publish cmd_vel topic
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // update timer
  rclcpp::TimerBase::SharedPtr timer_;

  void state_toggle_callback(const std_msgs::msg::String::SharedPtr msg);
};

extern std::shared_ptr<RobotStateCtrl> robot_state_ctrl;
