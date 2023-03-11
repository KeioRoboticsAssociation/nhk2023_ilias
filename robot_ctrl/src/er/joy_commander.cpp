// This node publishes a cmd_vel topic to the robot.
//  The topic is subscribed by the robot and the robot moves according to the
//  message.
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyCommander {
 public:
  JoyCommander(rclcpp::Node *ptr) : ros_ptr(ptr) {
    joy_cmd_vel.linear.x = 0;
    joy_cmd_vel.linear.y = 0;
    joy_cmd_vel.linear.z = 0;
    joy_cmd_vel.angular.x = 0;
    joy_cmd_vel.angular.y = 0;
    joy_cmd_vel.angular.z = 0;

    prev_cmd_vel.linear.x = 0;
    prev_cmd_vel.linear.y = 0;
    prev_cmd_vel.linear.z = 0;
    prev_cmd_vel.angular.x = 0;
    prev_cmd_vel.angular.y = 0;
    prev_cmd_vel.angular.z = 0;
  };
  geometry_msgs::msg::Twist joy_cmd_vel;
  geometry_msgs::msg::Twist prev_cmd_vel;
  sensor_msgs::msg::Joy current_joy;

  double max_linear_vel = 1;
  double max_angular_vel = 1;
  double max_linear_acc = 1;
  double max_angular_acc = 1;

  double prv_norm = 0;
  double current_norm = 0;
  double current_args = 0;

  int node_rate = 100;
  rclcpp::Node *ros_ptr;

  enum class button {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    BACK = 6,
    START = 7,
    HOME = 8,
    L3 = 9,
    R3 = 10,
  };

  enum class axis {
    LX = 0,
    LY = 1,
    LT = 4,
    RX = 2,
    RY = 3,
    RT = 5,
    DPAD_X = 6,
    DPAD_Y = 7,
  };

  void joy_callback(const sensor_msgs::msg::Joy &msg) { current_joy = msg; }

  void gen_cmd_vel() {
    // giving acceleration limit to joy_cmd_vel
    // null check

    if (current_joy.axes.size() == 0) return;

    RCLCPP_INFO(ros_ptr->get_logger(), "joy_cmd_vel: %f, %f, %f",
                joy_cmd_vel.linear.x, joy_cmd_vel.linear.y,
                joy_cmd_vel.angular.z);

    // current_norm = sqrt(pow(current_joy.axes[static_cast<int>(axis::LY)], 2)
    // +
    //                     pow(current_joy.axes[static_cast<int>(axis::LX)], 2))
    //                     *
    //                max_linear_vel;
    // current_args = atan2(current_joy.axes[static_cast<int>(axis::LX)],
    //                      current_joy.axes[static_cast<int>(axis::LY)]);

    // if(current_joy.axes[static_cast<int>(axis::LY)] == 0 &&
    //    current_joy.axes[static_cast<int>(axis::LX)] == 0){
    //   current_args = atan2(prev_cmd_vel.linear.y, prev_cmd_vel.linear.x);
    // }
    // // RCLCPP_INFO(ros_ptr->get_logger(), "current_norm: %f",
    // //             (current_norm - prv_norm));

    // if (current_norm - prv_norm > max_linear_acc) {
    //   // add velocity to joy_cmd_vel linearly using
    //   joy_cmd_vel.linear.x = prev_cmd_vel.linear.x +
    //                          max_linear_acc * cos(current_args) / node_rate;
    //   joy_cmd_vel.linear.y = prev_cmd_vel.linear.y +
    //                          max_linear_acc * sin(current_args) / node_rate;
    // } else if (current_norm - prv_norm < -max_linear_acc) {
    //   joy_cmd_vel.linear.x = prev_cmd_vel.linear.x -
    //                          max_linear_acc * cos(current_args) / node_rate;
    //   joy_cmd_vel.linear.y = prev_cmd_vel.linear.y -
    //                          max_linear_acc * sin(current_args) / node_rate;
    // } else {
    //   joy_cmd_vel.linear.x = current_norm * cos(current_args);
    //   joy_cmd_vel.linear.y = current_norm * sin(current_args);
    //   RCLCPP_INFO(ros_ptr->get_logger(), "###################");
    // }

    // if (current_joy.axes[static_cast<int>(axis::RX)] - joy_cmd_vel.angular.z
    // >
    //     max_angular_acc) {
    //   joy_cmd_vel.angular.z =
    //       joy_cmd_vel.angular.z + max_angular_acc / node_rate;
    // } else {
    //   joy_cmd_vel.angular.z =
    //       current_joy.axes[static_cast<int>(axis::RX)] * max_angular_vel;
    // }

    // add acceleration limit to joy_cmd_vel if it is over acc limit
    if (current_joy.axes[static_cast<int>(axis::LY)] * max_linear_vel -
            prev_cmd_vel.linear.x >
        max_linear_acc / node_rate) {
      joy_cmd_vel.linear.x = prev_cmd_vel.linear.x + max_linear_acc / node_rate;
    } else if (current_joy.axes[static_cast<int>(axis::LY)] * max_linear_vel -
                   prev_cmd_vel.linear.x <
               -max_linear_acc / node_rate) {
      joy_cmd_vel.linear.x = prev_cmd_vel.linear.x - max_linear_acc / node_rate;
    } else {
      joy_cmd_vel.linear.x =
          current_joy.axes[static_cast<int>(axis::LY)] * max_linear_vel;
    }

    if (current_joy.axes[static_cast<int>(axis::LX)] * max_linear_vel -
            prev_cmd_vel.linear.y >
        max_linear_acc / node_rate) {
      joy_cmd_vel.linear.y = prev_cmd_vel.linear.y + max_linear_acc / node_rate;
    } else if (current_joy.axes[static_cast<int>(axis::LX)] * max_linear_vel -
                   prev_cmd_vel.linear.y <
               -max_linear_acc / node_rate) {
      joy_cmd_vel.linear.y = prev_cmd_vel.linear.y - max_linear_acc / node_rate;
    } else {
      joy_cmd_vel.linear.y =
          current_joy.axes[static_cast<int>(axis::LX)] * max_linear_vel;
    }

    if (current_joy.axes[static_cast<int>(axis::RX)] * max_angular_vel -
            prev_cmd_vel.angular.z >
        max_angular_acc / node_rate) {
      joy_cmd_vel.angular.z =
          prev_cmd_vel.angular.z + max_angular_acc / node_rate;
    } else if (current_joy.axes[static_cast<int>(axis::RX)] * max_angular_vel -
                   prev_cmd_vel.angular.z <
               -max_angular_acc / node_rate) {
      joy_cmd_vel.angular.z =
          prev_cmd_vel.angular.z - max_angular_acc / node_rate;
    } else {
      joy_cmd_vel.angular.z =
          current_joy.axes[static_cast<int>(axis::RX)] * max_angular_vel;
    }

    // // update
    // prv_norm =
    //     sqrt(pow(joy_cmd_vel.linear.x, 2) + pow(joy_cmd_vel.linear.y, 2));
    prev_cmd_vel = joy_cmd_vel;
  }
};
