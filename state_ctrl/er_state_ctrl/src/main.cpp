#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

std::shared_ptr<RobotStateCtrl> robot_state_ctrl;
FSM_INITIAL_STATE(StateMachine, Start)

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  robot_state_ctrl = std::make_shared<RobotStateCtrl>();
  state_machine::start();
  rclcpp::spin(robot_state_ctrl);
  rclcpp::shutdown();
  return 0;
}
