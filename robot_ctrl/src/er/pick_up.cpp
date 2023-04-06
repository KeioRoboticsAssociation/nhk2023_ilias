#include <geometry_msgs/msg/twist.hpp>

#include "sensor.hpp"

const int BEHIND_RIGHT_SENSOR = 0;
const int BEHIND_LEFT_SENSOR = 1;
const int SIDE_RIGHT_TH0 = 0;  // 右サイドセンサーの１段階目
const int SIDE_RIGHT_TH1 = 1;  // 右サイドセンサーの２段階目
const int SIDE_LEFT_TH0 = 2;   // 左サイドセンサーの１段階目
const int SIDE_LEFT_TH1 = 3;   // 左サイドセンサーの２段階目

const u_int16_t SENSOR_MAX = 4095;
const float MAX_Y_VEL = 0.5;
const float SLOW_Y_VEL = 0.1;

class Pick_Up {
 private:
  std::unique_ptr<Sensor> sideSensor, behindSensor;

 public:
  Pick_Up(rclcpp::Node *ptr) {
    // サイド: デジタル測距
    sideSensor = std::make_unique<Sensor>(ptr, "side_sensor");
    // 後ろ: アナログ測距
    behindSensor = std::make_unique<Sensor>(ptr, "behind_sensor");
  }
  std::vector<int> behindVal, sideVal;
  // make prc_cmd_vel
  geometry_msgs::msg::Twist pick_up_cmd_vel;

  /*
   * @param isLeft true: left, false: right
   * @return true: finish, false: not finish
   */
  bool pick_up_vel_generator(bool isLeft) {
    behindVal = behindSensor->read();
    sideVal = sideSensor->read();
    // calc robot angular velocity from 2 sideSensor
    // migh be like p controller?
    pick_up_cmd_vel.angular.z =
        (behindVal[BEHIND_RIGHT_SENSOR] - behindVal[BEHIND_LEFT_SENSOR]) /
        SENSOR_MAX * 0.5;

    // just give constant velocity to y axis
    float y_vel = MAX_Y_VEL;
    if (isLeft) {
      if (sideVal[SIDE_LEFT_TH0])  // 閾値0を超えたら減速
        y_vel = SLOW_Y_VEL;
      else if (sideVal[SIDE_LEFT_TH1])  // 閾値1を超えたら停止
        y_vel = 0;
    } else {
      if (sideVal[SIDE_RIGHT_TH0])
        y_vel = SLOW_Y_VEL;
      else if (sideVal[SIDE_RIGHT_TH1])
        y_vel = 0;
    }

    pick_up_cmd_vel.linear.y = (isLeft) ? y_vel : -y_vel;

    // calc robot x velocity from 2 sideSensor
    // migh be like p controller?
    pick_up_cmd_vel.linear.x =
        ((behindVal[BEHIND_RIGHT_SENSOR] + behindVal[BEHIND_LEFT_SENSOR]) / 2 -
         1000) *
        0.5;
  }
};