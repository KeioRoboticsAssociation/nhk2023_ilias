#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Start::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Start entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "AUTO";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg auto");
  // publish state message
  msg.data = "START";
  robot_state_ctrl->state_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "state msg start");
}
