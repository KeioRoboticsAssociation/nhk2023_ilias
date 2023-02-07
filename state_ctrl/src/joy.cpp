#include "main.hpp"

void Joy::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Joy entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = 'JOY';
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg joy");
}