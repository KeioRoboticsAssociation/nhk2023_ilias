#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Last_Attack::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Last_Attack entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "LAST_ATTACK";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg last_attack");
  // publish state message
  msg.data = "LAST_ATTACK";
  robot_state_ctrl->state_pub_->publish(msg);
}