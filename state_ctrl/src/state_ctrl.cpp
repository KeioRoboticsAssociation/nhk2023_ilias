#include "state_ctrl.hpp"

#include "robot_state_ctrl.hpp"

StateMachine::StateMachine() {
  if (robot_state_ctrl == NULL) return;
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "StateMachine constructor");
}