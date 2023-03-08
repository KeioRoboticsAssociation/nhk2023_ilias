#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Angkor::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Angkor entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "ANGKOR";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg angkor");
}