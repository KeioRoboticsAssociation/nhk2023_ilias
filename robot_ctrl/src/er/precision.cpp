#include <geometry_msgs/msg/twist.hpp>

#include "sensor.hpp"

class Precision {
 private:
  std::unique_ptr<Sensor> sensor;

 public:
  Precision(rclcpp::Node *ptr) {
    sensor = std::make_unique<Sensor>(ptr, "sensor");
  }
  std::vector<int> sensor_val;
  // make prc_cmd_vel
  geometry_msgs::msg::Twist prc_cmd_vel;

  void precision_vel_generator() {
    sensor_val = sensor->read();
    if (sensor_val.size() < 4) {
      prc_cmd_vel.linear.x = 0;
      prc_cmd_vel.angular.z = 0.0;
      return;
    }
    if (sensor_val[3] > 13000) {
      prc_cmd_vel.linear.x = 0.5;
      prc_cmd_vel.angular.z = 0.0;
    } else if (sensor_val[3] <= 13000) {
      prc_cmd_vel.linear.x = 0;
      prc_cmd_vel.angular.z = 0.0;
    }
  }
};