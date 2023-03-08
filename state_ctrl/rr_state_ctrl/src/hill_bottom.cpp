#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Hill_Bottom::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Hill_Bottom entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "HILL_BOTTOM";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg hill_bottom");
}