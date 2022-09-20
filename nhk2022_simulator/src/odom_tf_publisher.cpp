#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

float position[2] = {};
geometry_msgs::msg::Quaternion odom_quat;

class Odom_tf_publisher : public rclcpp::Node {
 public:
  Odom_tf_publisher() : Node("odom_tf_publisher") {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

 private:
  void robot_odom_handler(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    geometry_msgs::msg::TransformStamped odom_trans;

    current_time = rclcpp::Time::now odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = base_frame_id;

    odom_quat.x = 0;
    odom_quat.y = 0;
    odom_quat.z = 0;
    odom_quat.w = 1;
  }

  void odomCallback(const nav_msgs::msg::Odometry& msg) {
    position[0] = msg.pose.pose.position.x;
    position[1] = msg.pose.pose.position.y;
    odom_quat = msg.pose.pose.orientation;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time = current_time;
  std::string base_frame_id = "base_link";
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ros::NodeHandle nh;
  ros::NodeHandle arg_n("~");

  std::string base_frame_id = "base_link";
  arg_n.getParam("base_frame_id", base_frame_id);

  odom_quat.x = 0;
  odom_quat.y = 0;
  odom_quat.z = 0;
  odom_quat.w = 1;

  ros::Subscriber sub = nh.subscribe("odom", 1, odomCallback);

  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time;
  current_time = ros::Time::now();

  ros::Rate r(30.0);

  while (nh.ok()) {
    current_time = ros::Time::now();

    // tf odom->base_link
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = base_frame_id;

    odom_trans.transform.translation.x = position[0];
    odom_trans.transform.translation.y = position[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    ros::spinOnce();
    r.sleep();
  }
}