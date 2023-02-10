// This node publishes a cmd_vel topic to the robot.
//  The topic is subscribed by the robot and the robot moves according to the
//  message.
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyCommander {
 public:
  JoyCommander(){};
  geometry_msgs::msg::Twist joy_cmd_vel;
  geometry_msgs::msg::Twist prev_cmd_vel;

  int max_linear_vel = 12;
  int max_angular_vel = 1;
  int max_linear_acc = 1;
  int max_angular_acc = 1;

  int node_rate = 100;

  enum class button {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    BACK = 6,
    START = 7,
    L3 = 8,
    R3 = 9,
  };

  enum class axis {
    LX = 0,
    LY = 1,
    LT = 2,
    RY = 3,
    RX = 4,
    RT = 5,
    DPAD_X = 6,
    DPAD_Y = 7,
  };

  void joy_callback(const sensor_msgs::msg::Joy &msg) {
    // giving acceleration limit to joy_cmd_vel
    // if (msg.axes[static_cast<int>(axis::LY)] * max_linear_vel -
    //         prev_cmd_vel.linear.x >
    //     max_linear_acc / node_rate) {
    //   joy_cmd_vel.linear.x = prev_cmd_vel.linear.x + max_linear_acc /
    //   node_rate;
    // } else if (msg.axes[static_cast<int>(axis::LY)] * max_linear_vel -
    //                prev_cmd_vel.linear.x <
    //            -max_linear_acc / node_rate) {
    //   joy_cmd_vel.linear.x = prev_cmd_vel.linear.x - max_linear_acc /
    //   node_rate;
    // } else {
    //   joy_cmd_vel.linear.x =
    //       msg.axes[static_cast<int>(axis::LY)] * max_linear_vel;
    // }

    // if (msg.axes[static_cast<int>(axis::LX)] * max_linear_vel -
    //         prev_cmd_vel.linear.y >
    //     max_linear_acc / node_rate) {
    //   joy_cmd_vel.linear.y = prev_cmd_vel.linear.y + max_linear_acc /
    //   node_rate;
    // } else if (msg.axes[static_cast<int>(axis::LX)] * max_linear_vel -
    //                prev_cmd_vel.linear.y <
    //            -max_linear_acc / node_rate) {
    //   joy_cmd_vel.linear.y = prev_cmd_vel.linear.y - max_linear_acc /
    //   node_rate;
    // } else {
    //   joy_cmd_vel.linear.y =
    //       msg.axes[static_cast<int>(axis::LX)] * max_linear_vel;
    // }

    // if (msg.axes[static_cast<int>(axis::RX)] * max_angular_vel -
    //         prev_cmd_vel.angular.z >
    //     max_angular_acc / node_rate) {
    //   joy_cmd_vel.angular.z =
    //       prev_cmd_vel.angular.z + max_angular_acc / node_rate;
    // } else if (msg.axes[static_cast<int>(axis::RX)] * max_angular_vel -
    //                prev_cmd_vel.angular.z <
    //            -max_angular_acc / node_rate) {
    //   joy_cmd_vel.angular.z =
    //       prev_cmd_vel.angular.z - max_angular_acc / node_rate;
    // } else {
    //   joy_cmd_vel.angular.z =
    //       msg.axes[static_cast<int>(axis::RX)] * max_angular_vel;
    // }
    if (msg.axes[static_cast<int>(axis::LY)] == 0) {
      joy_cmd_vel.linear.x = 0.0001;
    } else {
      joy_cmd_vel.linear.x =
          msg.axes[static_cast<int>(axis::LY)] * max_linear_vel;
    }

    // joy_cmd_vel.linear.y =
    //     msg.axes[static_cast<int>(axis::LX)] * max_linear_vel;
    // joy_cmd_vel.angular.z =
    //     msg.axes[static_cast<int>(axis::RX)] * max_angular_vel;
  }
};
