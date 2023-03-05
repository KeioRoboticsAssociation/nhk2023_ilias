#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class cmdvel2tf : public rclcpp::Node {
 public:
  cmdvel2tf() : Node("cmdvel2tf") {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&cmdvel2tf::callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

 private:
  rclcpp::Time last_time = this->now();

  void callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // callculate the frequency
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time).seconds();
    last_time = current_time;

    // get the transform from odom to base_link
    geometry_msgs::msg::TransformStamped odom2base;
    try {
      odom2base =
          tf_buffer_->lookupTransform("odom", "base_link", rclcpp::Time(0));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    // calculate the transform from odom to base_link
    geometry_msgs::msg::TransformStamped odom2base_new;
    odom2base_new.header.stamp = current_time;
    odom2base_new.header.frame_id = "odom";
    odom2base_new.child_frame_id = "base_link";
    odom2base_new.transform.translation.x =
        odom2base.transform.translation.x + msg->linear.x * dt;
    odom2base_new.transform.translation.y =
        odom2base.transform.translation.y + msg->linear.y * dt;
    odom2base_new.transform.translation.z =
        odom2base.transform.translation.z + msg->linear.z * dt;

    // add the rotation
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->angular.z * dt);
    geometry_msgs::msg::Quaternion q_msg;
    tf2::convert(q, q_msg);
    odom2base_new.transform.rotation = odom2base.transform.rotation * q_msg;

    // publish the transform
    br_->sendTransform(odom2base_new);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cmdvel2tf>());
  rclcpp::shutdown();
  return 0;
}