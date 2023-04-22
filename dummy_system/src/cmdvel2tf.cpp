#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class cmdvel2tf : public rclcpp::Node {
 public:
  cmdvel2tf() : Node("cmdvel2tf") {
    this->declare_parameter("base_frame_id", "base_link");
    this->declare_parameter("odom_frame_id", "odom");
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&cmdvel2tf::callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  }

 private:
  rclcpp::Time last_time = this->now();

  void callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // callculate the frequency
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time).seconds();
    last_time = current_time;
    RCLCPP_INFO(this->get_logger(), "dt: %f", dt);

    // get the transform from base_link to odom
    geometry_msgs::msg::TransformStamped odom2base;
    try {
      odom2base = tf_buffer_->lookupTransform(odom_frame_id_, base_frame_id_,
                                              rclcpp::Time(0));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    tf2::Quaternion q_old;
    tf2::convert(odom2base.transform.rotation, q_old);

    // convert quaternion to roll pitch yaw
    tf2::Matrix3x3 m(q_old);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // calculate the transform from odom to base_link
    geometry_msgs::msg::TransformStamped odom2base_new;
    odom2base_new.header.stamp = current_time;
    odom2base_new.header.frame_id = odom_frame_id_;
    odom2base_new.child_frame_id = base_frame_id_;
    odom2base_new.transform.translation.x =
        odom2base.transform.translation.x +
        (msg->linear.x * cos(yaw) - msg->linear.y * sin(yaw)) * dt;
    odom2base_new.transform.translation.y =
        odom2base.transform.translation.y +
        (msg->linear.x * sin(yaw) + msg->linear.y * cos(yaw)) * dt;
    odom2base_new.transform.translation.z = 0.0;

    // add the rotation
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->angular.z * dt);
    q = q_old * q;
    q.normalize();
    odom2base_new.transform.rotation = tf2::toMsg(q);

    // publish the transform
    br_->sendTransform(odom2base_new);

    // publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = odom2base_new.transform.translation.x;
    odom.pose.pose.position.y = odom2base_new.transform.translation.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom2base_new.transform.rotation;
    odom.twist.twist.linear.x = msg->linear.x;
    odom.twist.twist.linear.y = msg->linear.y;
    odom.twist.twist.angular.z = msg->angular.z;
    odometry_pub_->publish(odom);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  std::string base_frame_id_;
  std::string odom_frame_id_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cmdvel2tf>());
  rclcpp::shutdown();
  return 0;
}