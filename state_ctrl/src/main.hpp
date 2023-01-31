#pragma once
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "state_ctrl.hpp"

// ros class
class RobotStateCtrl : public rclcpp::Node {
 public:
  RobotStateCtrl() : Node("robot_state_ctrl") {
    RCLCPP_INFO(this->get_logger(), "RobotStateCtrl constructor");
  }
  ~RobotStateCtrl() {
    RCLCPP_INFO(this->get_logger(), "RobotStateCtrl destructor");
  }
};

extern std::shared_ptr<RobotStateCtrl> robot_state_ctrl;