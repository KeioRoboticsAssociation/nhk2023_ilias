#include <std_msgs/msg/int16.h>

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include "joy_commander.cpp"
#include "precision.cpp"

using namespace std::chrono_literals;

class robot_ctrl : public rclcpp::Node {
 public:
  robot_ctrl();
  // publish mode as topic
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  // subscribe joy topic
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  // subscribe mode topic
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  // publish cmd_vel topic
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // timer ptr
  rclcpp::TimerBase::SharedPtr timer_;

 private:
  enum class Mode {
    MANUAL,
    AUTO,
    CLIMB,
    PRECISION,
    IDLE,
  };

  JoyCommander joy_commander;
  Precision precision;

  Mode mode_, prev_mode_ = Mode::MANUAL;  // 初期モードはMANUAL
  void timer_callback();
  void mode_callback(const std_msgs::msg::String::SharedPtr msg);
};

robot_ctrl::robot_ctrl()
    : Node("rr_robot_ctrl"), joy_commander(this), precision(this) {
  RCLCPP_INFO(this->get_logger(), "robot_ctrl node is started");

  // set parameters
  this->declare_parameter("max_linear_vel", 12.0);
  this->declare_parameter("max_angular_vel", 3.0);
  this->declare_parameter("max_linear_acc", 3.0);
  this->declare_parameter("max_angular_acc", 3.0);

  // assign parameters
  joy_commander.max_linear_vel =
      this->get_parameter("max_linear_vel").as_double();
  joy_commander.max_angular_vel =
      this->get_parameter("max_angular_vel").as_double();
  joy_commander.max_linear_acc =
      this->get_parameter("max_linear_acc").as_double();
  joy_commander.max_angular_acc =
      this->get_parameter("max_angular_acc").as_double();

  // loop node at 100Hz
  timer_ = this->create_wall_timer(
      10ms, std::bind(&robot_ctrl::timer_callback, this));

  // subscribe joy topic
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoyCommander::joy_callback, &joy_commander,
                std::placeholders::_1));
  // subscribe mode topic
  mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "mode", 10,
      std::bind(&robot_ctrl::mode_callback, this, std::placeholders::_1));

  // publish cmd_vel topic
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // publish mode as topic
  mode_pub_ = this->create_publisher<std_msgs::msg::String>("cmd_vel_mode", 10);
}

void robot_ctrl::timer_callback() {
  // publish mode as topic
  auto msg = std_msgs::msg::String();
  // RCLCPP_INFO(this->get_logger(), "timer callback");

  switch (mode_) {
    case Mode::MANUAL:
      // publish cmd_vel topic
      joy_commander.gen_cmd_vel();
      cmd_vel_pub_->publish(joy_commander.joy_cmd_vel);
      msg.data = "MANUAL";
      // RCLCPP_INFO(this->get_logger(), "current mode manual");
      break;

    case Mode::AUTO:
      msg.data = "AUTO";
      RCLCPP_INFO(this->get_logger(), "current mode auto");
      break;

    case Mode::CLIMB:
      msg.data = "CLIMB";
      RCLCPP_INFO(this->get_logger(), "current mode climb");
      break;

    case Mode::PRECISION:
      precision.precision_vel_generator();
      RCLCPP_INFO(this->get_logger(), "sensor: %d", precision.sensor_val[3]);
      cmd_vel_pub_->publish(precision.prc_cmd_vel);
      msg.data = "PRECISION";
      RCLCPP_INFO(this->get_logger(), "current mode precision");
      break;

    case Mode::IDLE:
      msg.data = "IDLE";
      RCLCPP_INFO(this->get_logger(), "current mode idle");
      break;

    default:
      msg.data = "UNKNOWN";
      RCLCPP_INFO(this->get_logger(), "current mode unknown");
      break;
  }

  // publish mode as topic
  mode_pub_->publish(msg);

  // publish mode as topic only when mode is changed
  // if (mode_ != prev_mode_) {
  //   mode_pub_->publish(msg);
  //   prev_mode_ = mode_;
  // }
}

void robot_ctrl::mode_callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "mode changed to %s", msg->data.c_str());
  if (msg->data == "MANUAL") {
    mode_ = Mode::MANUAL;
  } else if (msg->data == "AUTO") {
    mode_ = Mode::AUTO;
  } else if (msg->data == "CLIMB") {
    mode_ = Mode::CLIMB;
  } else if (msg->data == "PRECISION") {
    mode_ = Mode::PRECISION;
  } else if (msg->data == "IDLE") {
    mode_ = Mode::IDLE;
  } else {
    mode_ = Mode::MANUAL;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_ctrl>());
  rclcpp::shutdown();
  return 0;
}
