#ifndef _MAIN_CPP_
#define _MAIN_CPP_

#include "main.hpp"

#include <rclcpp/rclcpp.hpp>

#include "state_ctrl.hpp"

std::shared_ptr<RobotStateCtrl> robot_state_ctrl;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  robot_state_ctrl = std::make_shared<RobotStateCtrl>();
  rclcpp::spin(robot_state_ctrl);
  rclcpp::shutdown();
  return 0;
}

#endif  // _MAIN_CPP_