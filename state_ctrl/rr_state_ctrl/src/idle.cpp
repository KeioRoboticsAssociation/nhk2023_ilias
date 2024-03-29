#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Idle::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Idle entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "IDLE";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg idle");
  // publish state message
  msg.data = "IDLE";
  robot_state_ctrl->state_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "state msg idle");
}
