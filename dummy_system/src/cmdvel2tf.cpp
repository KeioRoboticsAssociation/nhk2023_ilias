#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class cmdvel2tf : public rclcpp::Node {
 public:
  cmdvel2tf() : Node("cmdvel2tf") {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&cmdvel2tf::callback, this, std::placeholders::_1));
  }

 private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    static tf2_ros::TransformBroadcaster br(this);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    // get current position and add up the velocity
    

    tf2::Quaternion q;
    q.setRPY(msg->angular.x, msg->angular.y, msg->angular.z);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cmdvel2tf>());
  rclcpp::shutdown();
  return 0;
}