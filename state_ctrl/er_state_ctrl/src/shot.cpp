#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/twist.hpp>
#include <optional>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "robot_state_ctrl.hpp"
#include "state_ctrl.hpp"

void Shot::entry() {
  auto msg = std_msgs::msg::String();
  msg.data = "SHOT";
  robot_state_ctrl->mode_pub_->publish(msg);
  msg.data = "AUTO";
  robot_state_ctrl->state_pub_->publish(msg);
}

void Shot::react(Forward_Flag const& flag) { transit<PickupRight>(); }

void Shot::react(Update_Flag const& flag) {
  geometry_msgs::msg::TransformStamped robo2pole;
  try {
    robo2pole = robot_state_ctrl->tf_buffer_->lookupTransform(
        "base_link", "type3_pole", tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(robot_state_ctrl->get_logger(), "%s", ex.what());
  }
  float x = robo2pole.transform.translation.x;
  float y = robo2pole.transform.translation.y;
  float yaw = atan2(y, x);

  // RCLCPP_INFO(node->get_logger(), "x: %f, y: %f, yaw: %f", x, y, yaw);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.angular.z = yaw * 1;
  robot_state_ctrl->cmd_vel_pub_->publish(cmd_vel);
}