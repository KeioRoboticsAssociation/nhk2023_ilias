#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <optional>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"

const float aimGain = 1.0;

std::optional<geometry_msgs::msg::Twist> aim(
    rclcpp::Node* node, std::shared_ptr<tf2_ros::Buffer> tf_buffer_) {
  geometry_msgs::msg::TransformStamped robo2pole;
  try {
    robo2pole = tf_buffer_->lookupTransform("base_link", "type3_pole",
                                            tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(node->get_logger(), "%s", ex.what());
    return std::nullopt;
  }
  float x = robo2pole.transform.translation.x;
  float y = robo2pole.transform.translation.y;
  float yaw = atan2(y, x);

  RCLCPP_INFO(node->get_logger(), "x: %f, y: %f, yaw: %f", x, y, yaw);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.angular.z = yaw * 1;
  return cmd_vel;
}
