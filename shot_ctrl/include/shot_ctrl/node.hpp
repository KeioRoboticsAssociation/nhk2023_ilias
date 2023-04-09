#pragma once

#include <string>

#include "md_lib/md2022.hpp"
#include "md_lib/odrive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rogilink2_interfaces/msg/frame.hpp"

extern rclcpp::Node::SharedPtr node;

extern rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr rogilink;

void init();

template <typename... Args>
void logInfo(std::string msg, Args... args) {
  RCLCPP_INFO(node->get_logger(), msg.c_str(), args...);
}

template <typename... Args>
void logError(std::string msg, Args... args) {
  RCLCPP_ERROR(node->get_logger(), msg.c_str(), args...);
}