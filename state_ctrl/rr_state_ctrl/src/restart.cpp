#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Restart::entry(void) {
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "Restart entry");
  // publish mode message
  auto msg = std_msgs::msg::String();
  msg.data = "RESTART";
  robot_state_ctrl->mode_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "mode msg restart");
  // publish state message
  msg.data = "RESTART";
  robot_state_ctrl->state_pub_->publish(msg);
  RCLCPP_INFO(robot_state_ctrl->get_logger(), "state msg restart");

  robot_state_ctrl->led_->init();
  robot_state_ctrl->led_->setMode(Led::LedMessageId::Lighting, 8, 0,
                                  Led::LedColor::Green, 0, 0);
}