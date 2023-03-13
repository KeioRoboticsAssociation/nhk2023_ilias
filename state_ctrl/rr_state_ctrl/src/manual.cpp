#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Manual::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Manual entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "JOY";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg joy");
  // publish state message
  msg.data = "MANUAL";
  robot_state_ctrl->state_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "state msg manual");
}