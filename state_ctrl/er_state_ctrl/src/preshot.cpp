#include "geometry_msgs/msg/twist.hpp"
#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void PreShot::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "PRE_SHOT";
  robot_state_ctrl->state_pub_->publish(msg);
  msg.data = "AUTO";
  robot_state_ctrl->mode_pub_->publish(msg);

  auto cmdVel = geometry_msgs::msg::Twist();
  robot_state_ctrl->cmd_vel_pub_->publish(cmdVel);
  // 射出位置へ移動
  auto cmd = pure_pursuit_interface::msg::Frame();
  cmd.forward_flag = true;
  cmd.is_allowed_to_pub = true;
  cmd.path_num = 1;
  robot_state_ctrl->pure_pursuit_pub_->publish(cmd);
}

void PreShot::react(Forward_Flag const& flag) { transit<Shot>(); }

void PreShot::react(Update_Flag const& flag) {}