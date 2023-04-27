#include "pickupCourse.hpp"
#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void PickupRight::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "PICKUP_RIGHT";
  robot_state_ctrl->state_pub_->publish(msg);
  msg.data = "AUTO";
  robot_state_ctrl->mode_pub_->publish(msg);
  auto magazineMsg = std_msgs::msg::String();
  magazineMsg.data = "MAGAZINE_DOWN_RIGHT";
  robot_state_ctrl->shot_state_pub_->publish(magazineMsg);
  startSensing();
}

void PickupRight::exit() { stopSensing(); }

void PickupRight::react(Forward_Flag const&) {
  auto magazineMsg = std_msgs::msg::String();
  magazineMsg.data = "MAGAZINE_LOADED_RIGHT";
  robot_state_ctrl->shot_state_pub_->publish(magazineMsg);
  transit<PreShot>();
}

void PickupRight::react(Update_Flag const&) {
  bool finished = pickupVelGenerator(false);
  // if (finished) transit<PreShot>();
}