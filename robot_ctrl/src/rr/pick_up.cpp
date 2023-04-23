#include <geometry_msgs/msg/twist.hpp>

#include "limitSwitch.hpp"

class PickUp {
 private:
  std::unique_ptr<LimitSwitch> sensor;
  // sensor data array
  std::array<uint8_t, 4> sensor_data;

  // sensor callback
  void sensor_callback(int index) { sensor_data[index] = true; }

 public:
  PickUp(rclcpp::Node *ptr) {
    sensor = std::make_unique<LimitSwitch>(ptr, "sensor");
    for (int i = 0; i < 4; i++) {
      sensor->addCallback(i, std::function<void()>(
                                 std::bind(&PickUp::sensor_callback, this, i)));
    }
  }

  // make prc_cmd_vel
  geometry_msgs::msg::Twist pick_cmd_vel;
  int stop_count = 0;

  void gen_pick_vel() {
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
        stop_count = 0;
      }
    }
  }
};