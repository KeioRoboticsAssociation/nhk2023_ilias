#include "pickupCourse.hpp"
#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void PickupRight::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "PICKUP_RIGHT";
  robot_state_ctrl->state_pub_->publish(msg);
  msg.data = "AUTO";
  robot_state_ctrl->mode_pub_->publish(msg);
}

void PickupRight::react(Forward_Flag const& flag) { transit<PreShot>(); }

void PickupRight::exit() { stopSensing(); }

void PickupRight::react(Update_Flag const&) {
  bool finished = pickupVelGenerator(true);
  // if (finished) transit<PreShot>();
}