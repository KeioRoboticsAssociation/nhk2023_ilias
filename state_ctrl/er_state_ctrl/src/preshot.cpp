#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void PreShot::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "PRE_SHOT";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg pre_shot");
}

void PreShot::react(Forward_Flag const& flag) { transit<Shot>(); }