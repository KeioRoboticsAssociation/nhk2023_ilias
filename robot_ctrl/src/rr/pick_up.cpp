#include <geometry_msgs/msg/twist.hpp>

#include "sensor_lib/include/limitSwitch.hpp"

class PickUp {
 private:
  std::unique_ptr<LimitSwitch> sensor;
  // sensor data array
  std::array<uint8_t, 4> sensor_data;

 public:
  PickUp(rclcpp::Node *ptr) {
    sensor = std::make_unique<Sensor>(ptr, "sensor");
    for (int i = 0; i < 4; i++) {
      sensor->addCallback(i, []() {
        for (int i = 0; i < 4; i++) {
          sensor_data[i] = sensor->getSensorData(i);
        }
        sensor_data[i] = true;
      });
    }
  }

  // make prc_cmd_vel
  geometry_msgs::msg::Twist pick_cmd_vel;
  int stop_count = 0;

  void pick_vel_generator() {
    if (sensor_data[0] == true) {
      if (sensor_data[2] == true) {
        pick_cmd_vel.linear.y = 0.1;
        stop_count++;
      } else {
        pick_cmd_vel.linear.y = 0.2;
      }
    } else {
      pick_cmd_vel.linear.y = 0.3;
    }
    // stop after 500ms and move forward until sensor is detected
    if (stop_count > 50) {
      pick_cmd_vel.linear.y = 0.0;
      if (sensor_data[1] == true) {
        pick_cmd_vel.linear.x = 0.1;
      } else {
        pick_cmd_vel.linear.x = 0;
        end_flag = true;
        stop_count = 0;
      }
    }
  }
};