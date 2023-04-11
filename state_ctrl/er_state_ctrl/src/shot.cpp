#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Shot::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "SHOT";
  robot_state_ctrl->mode_pub_->publish(msg);
  msg.data = "AUTO";
  robot_state_ctrl->state_pub_->publish(msg);
}

void Shot::react(Forward_Flag const& flag) { transit<PickupRight>(); }

void Shot::react(Update_Flag const& flag) {}