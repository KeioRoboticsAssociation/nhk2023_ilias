#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void PickupRight::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "PICKUP_RIGHT";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg pickupRight");
}

void PickupRight::react(Forward_Flag const& flag) { transit<PreShot>(); }