// #pragma once
#include "robot_state_ctrl.hpp"

#include <memory>

#include "pure_pursuit_interface/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state_ctrl.hpp"
#include "std_msgs/msg/empty.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

RobotStateCtrl::RobotStateCtrl() : Node("er_state_ctrl") {
  RCLCPP_INFO(this->get_logger(), "RobotStateCtrl");
  // subscribe mode changer as topic
  state_toggle_sub_ = this->create_subscription<std_msgs::msg::String>(
      "state_toggle", 10,
      std::bind(&RobotStateCtrl::state_toggle_callback, this,
                std::placeholders::_1));

  // publish mode as topic
  mode_pub_ = this->create_publisher<std_msgs::msg::String>("mode", 10);
  // puclish state as topic
  state_pub_ = this->create_publisher<std_msgs::msg::String>("state", 10);
  // publish cmd_vel topic
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // サイド: デジタル測距
  sideSensor = std::make_unique<Sensor>(this, "side_sensor");
  // 後ろ: アナログ測距
  behindSensor = std::make_unique<Sensor>(this, "behind_sensor");
  // publish pure_pursuit topic
  pure_pursuit_pub_ =
      this->create_publisher<pure_pursuit_interface::msg::Frame>("pp_cmd", 10);
  // subscribe pursuit_end topic
  pursuit_end_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "pp_end", 10, [=](const std_msgs::msg::Empty::SharedPtr) {
        auto disableCmd = pure_pursuit_interface::msg::Frame();
        disableCmd.is_allowed_to_pub = false;
        pure_pursuit_pub_->publish(disableCmd);

        state_machine::dispatch(Forward_Flag());
      });

  robot_state_ctrl->timer_ = this->create_wall_timer(
      10ms, [=]() { state_machine::dispatch(Update_Flag()); });
  // RCLCPP_INFO(this->get_logger(), "RobotStateCtrl constructor end");
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
  } else if (msg->data == "START") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "START");
    state_machine::dispatch(Start_Flag());
  } else if (msg->data == "FORWARD") {
    RCLCPP_INFO(robot_state_ctrl->get_logger(), "FORWARD");
    state_machine::dispatch(Forward_Flag());
  }
}
