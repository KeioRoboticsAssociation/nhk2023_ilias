#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Angkor_Center::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Angkor_Center entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "ANGKOR_CENTER";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg angkor_center");
  // publish state message
  msg.data = "ANGKOR_CENTER";
  robot_state_ctrl->state_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "state msg angkor_center");
}