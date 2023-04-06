#include <geometry_msgs/msg/twist.hpp>

#include "sensor.hpp"

const int RIGHT_SENSOR = 0;
const int LEFT_SENSOR = 1;
const u_int16_t SENSOR_MAX = 4095;

class Pick_Up {
 private:
  std::unique_ptr<Sensor> sensor;

 public:
  Pick_Up(rclcpp::Node *ptr) {
    sensor = std::make_unique<Sensor>(ptr, "sensor");
  }
  std::vector<int> sensor_val;
  // make prc_cmd_vel
  geometry_msgs::msg::Twist pick_up_cmd_vel;

  void pick_up_vel_generator(bool is_pick_up, bool isLeft) {
    sensor_val = sensor->read();
    // calc robot angular velocity from 2 sensor
    // migh be like p controller?
    pick_up_cmd_vel.angular.z =
        (sensor_val[RIGHT_SENSOR] - sensor_val[LEFT_SENSOR]) / SENSOR_MAX * 0.5;
    // just give constant velocity to y axis
    if (isLeft) {
      if (is_pick_up) {
        pick_up_cmd_vel.linear.y = 0.5;
      } else {
        pick_up_cmd_vel.linear.y = -0.5;
      }
    } else {
      if (is_pick_up) {
        pick_up_cmd_vel.linear.y = -0.5;
      } else {
        pick_up_cmd_vel.linear.y = 0.5;
      }
    }
    // calc robot x velocity from 2 sensor
    // migh be like p controller?
    pick_up_cmd_vel.linear.x =
        ((sensor_val[RIGHT_SENSOR] + sensor_val[LEFT_SENSOR]) / 2 - 1000) * 0.5;
  }
};