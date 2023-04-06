#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Shot::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "SHOT";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg shot");
}

void Shot::react(Forward_Flag const& flag) { transit<PickupRight>(); }