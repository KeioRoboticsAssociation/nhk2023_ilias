#include "main.hpp"

void Idle::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Idle entry");
}
