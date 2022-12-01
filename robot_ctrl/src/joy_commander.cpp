// This node publishes a cmd_vel topic to the robot.
//  The topic is subscribed by the robot and the robot moves according to the
//  message.

#include "joy_commander.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyCommander : public rclcpp::Node {
 public:
  JoyCommander();
  geometry_msgs::msg::Twist cmd_vel;

 private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // timer in 10Hz
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
};

JoyCommander::JoyCommander() : Node("joy_commander") {
  RCLCPP_INFO(this->get_logger(), "joy_commander node is started");

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoyCommander::joy_callback, this, std::placeholders::_1));

  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&JoyCommander::timer_callback, this));
}

void JoyCommander::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  cmd_vel.linear.x = msg->axes[static_cast<int>(JoyCommander::axes::LY)];
  cmd_vel.linear.y = msg->axes[static_cast<int>(JoyCommander::axes::LX)];
  cmd_vel.angular.z = msg->axes[static_cast<int>(JoyCommander::axes::RX)];
}

void JoyCommander::timer_callback() { cmd_vel_pub_->publish(cmd_vel); }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyCommander>());
  rclcpp::shutdown();
  return 0;
}
