#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void PickupLeft::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "PICKUP_LEFT";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg pickupLeft");
}

void PickupLeft::react(Forward_Flag const&) { transit<PreShot>(); }