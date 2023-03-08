#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Hill_Top::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Hill_Top entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "HILL_TOP";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg hill_top");
}