// #pragma once
#include "robot_state_ctrl.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "state_ctrl.hpp"
#include "std_msgs/msg/string.hpp"

RobotStateCtrl::RobotStateCtrl() : Node("rr_state_ctrl") {
  RCLCPP_INFO(this->get_logger(), "RobotStateCtrl");
  RCLCPP_INFO(this->get_logger(), "RobotStateCtrl constructor start");
  // subscribe mode changer as topic
  state_toggle_sub_ = this->create_subscription<std_msgs::msg::String>(
      "state_toggle", 10,
      std::bind(&RobotStateCtrl::state_toggle_callback, this,
                std::placeholders::_1));
  // publish state
  state_pub_ = this->create_publisher<std_msgs::msg::String>("state", 10);
  // publish cmd_vel mode as topic
  mode_pub_ = this->create_publisher<std_msgs::msg::String>("mode", 10);
  // publish pure pursuit command as topic
  pure_pursuit_pub_ = this->create_publisher<pure_pursuit_interface::msg::Frame>(
      "pp_cmd", 10);

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
  } else if (msg->data == "START") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "START");
    state_machine::dispatch(GOD_Flag(), "START");
  } else if (msg->data == "RESTART") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "RESTART");
    state_machine::dispatch(GOD_Flag(), "RESTART");
  } else if (msg->data == "HILL_BOTTOM") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "HILL_BOTTOM");
    state_machine::dispatch(GOD_Flag(), "HILL_BOTTOM");
  } else if (msg->data == "HILL_TOP") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "HILL_TOP");
    state_machine::dispatch(GOD_Flag(), "HILL_TOP");
  } else if (msg->data == "ANGKOR") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "ANGKOR");
    state_machine::dispatch(GOD_Flag(), "ANGKOR");
  } else if (msg->data == "ANGKOR_CENTER") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "ANGKOR_CENTER");
    state_machine::dispatch(GOD_Flag(), "ANGKOR_CENTER");
  } else if (msg->data == "TYPE2_ATTACK") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "TYPE2_ATTACK");
    state_machine::dispatch(GOD_Flag(), "TYPE2_ATTACK");
  } else if (msg->data == "POLE_BLOCK") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "POLE_BLOCK");
    state_machine::dispatch(GOD_Flag(), "POLE_BLOCK");
  } else if (msg->data == "LAST_ATTACK") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "LAST_ATTACK");
    state_machine::dispatch(GOD_Flag(), "LAST_ATTACK");
  } else if (msg->data == "END") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "END");
    state_machine::dispatch(GOD_Flag(), "END");
  }
}
