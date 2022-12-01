// header file of joy_commander.cpp

#ifndef JOY_COMMANDER_HPP_
#define JOY_COMMANDER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyCommander : public rclcpp::Node {
 public:
  JoyCommander();
  geometry_msgs::msg::Twist cmd_vel;

  enum class axes {
    LX,
    LY,
    RX,
    RY,
  };

  enum class buttons {
    A,
    B,
    X,
    Y,
    LB,
    RB,
    BACK,
    START,
    LOGITECH,
    LX,
    LY,
  };

 private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // timer in 10Hz
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
};

#endif  // JOY_COMMANDER_HPP_
