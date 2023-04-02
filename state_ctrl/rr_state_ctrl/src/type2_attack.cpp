#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Type2_Attack::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Type2_Attack entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "TYPE2_ATTACK";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg type2_attack");
  // publish state message
  msg.data = "TYPE2_ATTACK";
  robot_state_ctrl->state_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "state msg type2_attack");
}