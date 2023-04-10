#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>

#include "sensor.hpp"

const int BEHIND_RIGHT_SENSOR = 3;
const int BEHIND_LEFT_SENSOR = 2;
const int SIDE_RIGHT_TH0 = 1;  // 右サイドセンサーの１段階目
const int SIDE_RIGHT_TH1 = 0;  // 右サイドセンサーの２段階目
const int SIDE_LEFT_TH0 = 3;   // 左サイドセンサーの１段階目
const int SIDE_LEFT_TH1 = 2;   // 左サイドセンサーの２段階目

const int SENSOR_MAX = 65535;
const int TARGET_DISTANCE = 20000;
const float MAX_Y_VEL = 0.5;
const float SLOW_Y_VEL = 0.1;

class Pick_Up {
 private:
  std::unique_ptr<Sensor> sideSensor, behindSensor;
  rclcpp::Node *ptr;

 public:
  Pick_Up(rclcpp::Node *ptr) : ptr(ptr) {
    // サイド: デジタル測距
    sideSensor = std::make_unique<Sensor>(ptr, "side_sensor");
    // 後ろ: アナログ測距
    behindSensor = std::make_unique<Sensor>(ptr, "behind_sensor");
  }
  std::vector<int> behindVal, sideVal;
  // make prc_cmd_vel
  geometry_msgs::msg::Twist pick_up_cmd_vel;

  void startSensing() {
    sideSensor->start();
    behindSensor->start();
  }

  void stopSensing() {
    sideSensor->stop();
    behindSensor->stop();
  }

  /*
   * @param isLeft true: left, false: right
   * @return true: finish, false: not finish
   */
  bool pick_up_vel_generator(bool isLeft) {
    static int prevBehindL, prevBehindR;
    behindVal = behindSensor->read();
    sideVal = sideSensor->read();

    if (behindVal.size() < 4 || sideVal.size() < 4) {
      RCLCPP_ERROR(ptr->get_logger(), "sensor read error");
      return false;
    }

    int behindL = 0.1 * behindVal[BEHIND_LEFT_SENSOR] + 0.9 * prevBehindL;
    int behindR = 0.1 * behindVal[BEHIND_RIGHT_SENSOR] + 0.9 * prevBehindR;

    prevBehindL = behindL;
    prevBehindR = behindR;

    behindL += 800;

    RCLCPP_INFO(ptr->get_logger(), "behind: %d, %d", behindL, behindR);
    // RCLCPP_INFO(ptr->get_logger(), "side: %d, %d, %d, %d", sideVal[0],
    // sideVal[1], sideVal[2], sideVal[3]);
    // calc robot angular velocity from 2 sideSensor
    // migh be like p controller?
    pick_up_cmd_vel.angular.z =
        std::clamp(-(double)(behindR - behindL) / SENSOR_MAX * 10, -1.27, 1.27);

    // just give constant velocity to y axis
    float y_vel = MAX_Y_VEL;
    if (isLeft) {
      if (sideVal[SIDE_LEFT_TH0] > 10000)  // 閾値0を超えたら減速
        y_vel = SLOW_Y_VEL;
      if (sideVal[SIDE_LEFT_TH1] > 10000)  // 閾値1を超えたら停止
        y_vel = 0;
    } else {
      if (sideVal[SIDE_RIGHT_TH0] > 10000) y_vel = SLOW_Y_VEL;
      if (sideVal[SIDE_RIGHT_TH1] > 10000) y_vel = 0;
    }

    pick_up_cmd_vel.linear.y = (isLeft) ? y_vel : -y_vel;

    // calc robot x velocity from 2 sideSensor
    // migh be like p controller?
    pick_up_cmd_vel.linear.x = std::clamp(
        -(float)((behindR + behindL) / 2 - TARGET_DISTANCE) * 0.0005f, -1.0f,
        1.0f);

    // RCLCPP_INFO(ptr->get_logger(), "x: %f, yaw: %f",
    // pick_up_cmd_vel.linear.x,
    //             pick_up_cmd_vel.angular.z);
    if (y_vel == 0) return 1;
    return 0;
  }
};