// #pragma once
#include "robot_state_ctrl.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "state_ctrl.hpp"
#include "std_msgs/msg/string.hpp"

RobotStateCtrl::RobotStateCtrl() : Node("er_state_ctrl") {
  RCLCPP_INFO(this->get_logger(), "RobotStateCtrl");
  RCLCPP_INFO(this->get_logger(), "RobotStateCtrl constructor start");
  // subscribe mode changer as topic
  state_toggle_sub_ = this->create_subscription<std_msgs::msg::String>(
      "state_toggle", 10,
      std::bind(&RobotStateCtrl::state_toggle_callback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "RobotStateCtrl constructor wei");
  // publish mode as topic
  mode_pub_ = this->create_publisher<std_msgs::msg::String>("mode", 10);

  RCLCPP_INFO(this->get_logger(), "RobotStateCtrl constructor end");
}

void RobotStateCtrl::state_toggle_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "state_toggle_callback");
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "I heard: [%s]",
              msg->data.c_str());
  if (msg->data == "MANUAL") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "MANUAL");
    state_machine::dispatch(Manual_Flag());
  } else if (msg->data == "IDLE") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "IDLE");
    state_machine::dispatch(Idle_Flag());
  } else if (msg->data == "FORWARD") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "FORWARD");
    state_machine::dispatch(Forward_Flag());
  }
}
