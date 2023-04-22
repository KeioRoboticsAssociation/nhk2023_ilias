#pragma once
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "pure_pursuit_interface/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor.hpp"
#include "state_ctrl.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

// ros class
class RobotStateCtrl : public rclcpp::Node {
 public:
  RobotStateCtrl();
  ~RobotStateCtrl() {}
  // 走行モードのpublisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  // ステートのpublisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  // subscribe mode changer as topic
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_toggle_sub_;
  // publish cmd_vel topic
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // update timer
  rclcpp::TimerBase::SharedPtr timer_;
  // tf buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // tf listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // Distanse sensor
  std::unique_ptr<Sensor> sideSensor, behindSensor;
  // publish pure_pursuit topic
  rclcpp::Publisher<pure_pursuit_interface::msg::Frame>::SharedPtr
      pure_pursuit_pub_;
  // subscribe pursuit_end topic
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pursuit_end_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_state_pub_;

  void state_toggle_callback(const std_msgs::msg::String::SharedPtr msg);
};

extern std::shared_ptr<RobotStateCtrl> robot_state_ctrl;
