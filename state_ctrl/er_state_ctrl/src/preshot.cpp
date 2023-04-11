#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void PreShot::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "PRE_SHOT";
  robot_state_ctrl->state_pub_->publish(msg);
  msg.data = "AUTO";
  robot_state_ctrl->mode_pub_->publish(msg);
}

void PreShot::react(Forward_Flag const& flag) { transit<Shot>(); }

void PreShot::react(Update_Flag const& flag) {}