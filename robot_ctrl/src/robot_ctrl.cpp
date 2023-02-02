#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include "joy_commander.cpp"

using namespace std::chrono_literals;

class robot_ctrl : public rclcpp::Node, public JoyCommander {
 private:
  enum class Mode {
    MANUAL,
    AUTO,
    CLIMB,
    PRECISION,
  };

  Mode mode_, prev_mode_ = Mode::MANUAL;  // 初期モードはMANUAL
  void timer_callback();

 public:
  robot_ctrl();

  // publish mode as topic
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  // subscribe joy topic
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  // publish cmd_vel topic
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // timer ptr
  rclcpp::TimerBase::SharedPtr timer_;
};

robot_ctrl::robot_ctrl() : Node("robot_ctrl") {
  RCLCPP_INFO(this->get_logger(), "robot_ctrl node is started");
  // loop node at 10Hz
  timer_ = this->create_wall_timer(
      100ms, std::bind(&robot_ctrl::timer_callback, this));
  // publish mode as topic
  mode_pub_ = this->create_publisher<std_msgs::msg::String>("mode", 10);
  // subscribe joy topic
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&robot_ctrl::joy_callback, this, std::placeholders::_1));
  // publish cmd_vel topic
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void robot_ctrl::timer_callback() {
  // publish mode as topic
  auto msg = std_msgs::msg::String();
  RCLCPP_INFO(this->get_logger(), "timer callback");

  switch (mode_) {
    case Mode::MANUAL:
      // publish cmd_vel topic
      cmd_vel_pub_->publish(JoyCommander::joy_cmd_vel);
      msg.data = "MANUAL";
      break;

    case Mode::AUTO:
      msg.data = "AUTO";
      break;

    case Mode::CLIMB:
      msg.data = "CLIMB";
      break;

    case Mode::PRECISION:
      msg.data = "PRECISION";
      break;

    default:
      msg.data = "UNKNOWN";
      break;
  }
  mode_pub_->publish(msg);
  // publish mode as topic only when mode is changed
  if (mode_ != prev_mode_) {
    mode_pub_->publish(msg);
    prev_mode_ = mode_;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_ctrl>());
  rclcpp::shutdown();
  return 0;
}
