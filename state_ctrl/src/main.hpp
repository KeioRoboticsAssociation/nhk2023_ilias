#pragma once
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "state_ctrl.hpp"
#include "std_msgs/msg/string.hpp"

// ros class
class RobotStateCtrl : public rclcpp::Node {
 public:
  RobotStateCtrl() : Node("robot_state_ctrl") {
    RCLCPP_INFO(this->get_logger(), "RobotStateCtrl constructor");
  }
  ~RobotStateCtrl() {
    RCLCPP_INFO(this->get_logger(), "RobotStateCtrl destructor");
  }
  // publish mode as topic
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
};

extern std::shared_ptr<RobotStateCtrl> robot_state_ctrl;