#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void End::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "End entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "END";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg end");
  // publish state message
  msg.data = "END";
  robot_state_ctrl->state_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "state msg end");
  
}