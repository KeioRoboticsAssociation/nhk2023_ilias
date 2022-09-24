#include <memory>
#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class Odom_tf_publisher : public rclcpp::Node {
 public:
  Odom_tf_publisher() : Node("odom_tf_publisher") {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(10),
        std::bind(&Odom_tf_publisher::OdomCallback, this,
                  std::placeholders::_1));

    this->declare_parameter<std::string>("base_frame_id", "base_link");
    this->declare_parameter<int>("loop_rate", 30);
    this->get_parameter<std::string>("base_frame_id", base_frame_id);
    this->get_parameter<int>("loop_rate", loop_rate);

    odom_quat.x = 0;
    odom_quat.y = 0;
    odom_quat.z = 0;
    odom_quat.w = 1;

    rclcpp::WallRate loop_rate(this->loop_rate);

    while (rclcpp::ok) {
      robot_odom_handler();
      loop_rate.sleep();
    }
  }

 private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  int loop_rate = 30;

  std::string base_frame_id = "base_link";
  geometry_msgs::msg::TransformStamped odom_trans;

  float position[2] = {};
  geometry_msgs::msg::Quaternion odom_quat;

  void OdomCallback(const nav_msgs::msg::Odometry& msg) {
    position[0] = msg.pose.pose.position.x;
    position[1] = msg.pose.pose.position.y;
    odom_quat = msg.pose.pose.orientation;
    robot_odom_handler();
  }

  void robot_odom_handler() {
    odom_trans.header.stamp = this->get_clock()->now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = base_frame_id;

    odom_trans.transform.translation.x = position[0];
    odom_trans.transform.translation.y = position[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    tf_broadcaster_->sendTransform(odom_trans);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odom_tf_publisher>(argv));
  rclcpp::shutdown();

  return 0;
}