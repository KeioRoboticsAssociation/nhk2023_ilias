#include "pickupCourse.hpp"

const int BEHIND_RIGHT_SENSOR = 1;
const int BEHIND_LEFT_SENSOR = 2;
const int SIDE_RIGHT_TH0 = 1;  // 右サイドセンサーの１段階目
const int SIDE_RIGHT_TH1 = 0;  // 右サイドセンサーの２段階目
const int SIDE_LEFT_TH0 = 3;   // 左サイドセンサーの１段階目
const int SIDE_LEFT_TH1 = 2;   // 左サイドセンサーの２段階目

const int SENSOR_MAX = 65535;
const int TARGET_DISTANCE = 13500;
const float MAX_Y_VEL = 1.3;
const float SLOW_Y_VEL = 0.3;

void startSensing() {
  robot_state_ctrl->behindSensor->start();
  robot_state_ctrl->sideSensor->start();
}

void stopSensing() {
  robot_state_ctrl->sideSensor->stop();
  robot_state_ctrl->behindSensor->stop();
}

bool pickupVelGenerator(bool isLeft) {
  geometry_msgs::msg::Twist pick_up_cmd_vel;

  static int prevBehindL, prevBehindR;
  auto behindVal = robot_state_ctrl->behindSensor->read();
  auto sideVal = robot_state_ctrl->sideSensor->read();

  if (behindVal.size() < 4 || sideVal.size() < 4) {
    RCLCPP_ERROR(robot_state_ctrl->get_logger(), "sensor read error");
    return false;
  }

  int behindL = 0.05 * behindVal[BEHIND_LEFT_SENSOR] + 0.95 * prevBehindL;
  int behindR = 0.05 * behindVal[BEHIND_RIGHT_SENSOR] + 0.95 * prevBehindR;

  prevBehindL = behindL;
  prevBehindR = behindR;

  behindL += 800;

  RCLCPP_INFO(robot_state_ctrl->get_logger(), "behind: %d, %d", behindL,
              behindR);
  // RCLCPP_INFO(ptr->get_logger(), "side: %d, %d, %d, %d", sideVal[0],
  // sideVal[1], sideVal[2], sideVal[3]);
  // calc robot angular velocity from 2 sideSensor
  // migh be like p controller?

  // just give constant velocity to y axis
  float y_vel = MAX_Y_VEL;
  if (isLeft) {
    if (sideVal[SIDE_LEFT_TH0] > 10000) {  // 閾値0を超えたら減速
      y_vel = SLOW_Y_VEL;
    }
    if (sideVal[SIDE_LEFT_TH1] > 10000)  // 閾値1を超えたら停止
      y_vel = 0;
  } else {
    if (sideVal[SIDE_RIGHT_TH0] > 10000) y_vel = SLOW_Y_VEL;
    if (sideVal[SIDE_RIGHT_TH1] > 10000) y_vel = 0;
  }

  pick_up_cmd_vel.linear.y = (isLeft) ? y_vel : -y_vel;

  // calc robot x 1e0oci1y0from 2 sideSensor
  // migh be like p controller?
  pick_up_cmd_vel.linear.x =
      std::clamp(-(float)((behindR + behindL) / 2 - TARGET_DISTANCE) * 0.0002f,
                 -1.0f, 1.0f);
  float yawGain = (y_vel == 0) ? 15 : 8;
  pick_up_cmd_vel.angular.z = std::clamp(
      -(double)(behindR - behindL) / SENSOR_MAX * yawGain, -1.27, 1.27);

  // RCLCPP_INFO(ptr->get_logger(), "x: %f, yaw: %f",
  // pick_up_cmd_vel.linear.x,
  //             pick_up_cmd_vel.angular.z);
  robot_state_ctrl->cmd_vel_pub_->publish(pick_up_cmd_vel);

  if (y_vel == 0) {
    return 1;
  }
  return 0;
}