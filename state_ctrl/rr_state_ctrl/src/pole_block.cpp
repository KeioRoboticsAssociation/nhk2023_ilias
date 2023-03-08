#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Pole_Block::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Pole_Block entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "POLE_BLOCK";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg pole_block");
}