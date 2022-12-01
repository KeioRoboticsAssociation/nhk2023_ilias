#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include "joy_commander.hpp"

class robot_ctrl : public rclcpp::Node {
 private:
  enum class Mode {
    MANUAL,
    AUTO,
    CLIMB,
    PRECISION,
  };

  Mode mode_ = Mode::MANUAL;

 public:
  robot_ctrl();
};

robot_ctrl::robot_ctrl() : Node("robot_ctrl") {
  RCLCPP_INFO(this->get_logger(), "robot_ctrl node is started");
  switch (mode_) {
    case Mode::MANUAL:
      RCLCPP_INFO(this->get_logger(), "Mode: MANUAL");
      break;

    default:
      break;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_ctrl>());
  rclcpp::shutdown();
  return 0;
}
