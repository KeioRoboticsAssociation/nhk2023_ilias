#include "../include/shot_ctrl/node.hpp"

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr rogilink;

void init() {
  node = rclcpp::Node::make_shared("shot_ctrl");
  rogilink = node->create_publisher<rogilink2_interfaces::msg::Frame>(
      "rogilink2/send", 10);
}
