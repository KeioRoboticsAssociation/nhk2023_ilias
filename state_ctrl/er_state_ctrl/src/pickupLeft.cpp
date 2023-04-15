#include "pickupCourse.hpp"
#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void PickupLeft::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "PICKUP_LEFT";
  robot_state_ctrl->state_pub_->publish(msg);
  msg.data = "AUTO";
  robot_state_ctrl->mode_pub_->publish(msg);
  startSensing();
}

void PickupLeft::exit() { stopSensing(); }

void PickupLeft::react(Forward_Flag const &) { transit<PreShot>(); }

void PickupLeft::react(Update_Flag const &) {
  bool finished = pickupVelGenerator(true);
  // if (finished) transit<PreShot>();
}