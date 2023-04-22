#pragma once

#include <string>

#include "limitSwitch.hpp"
#include "md_lib/md2022.hpp"
#include "md_lib/odrive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rogilink2_interfaces/msg/frame.hpp"
#include "sensor.hpp"

extern rclcpp::Node::SharedPtr node;

extern rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr rogilink;

extern std::shared_ptr<LimitSwitch> limitSensor;

extern std::shared_ptr<MD2022> leftMagazine;
extern std::shared_ptr<MD2022> rightMagazine;
extern std::shared_ptr<MD2022> leftPusher;
extern std::shared_ptr<MD2022> rightPusher;
extern std::shared_ptr<MD2022> loader;
extern std::shared_ptr<MD2022> elevation;
extern std::shared_ptr<ODrive> shooter;

const int LEFT_MAGAZIN_LIMIT = 0;
const int RIGHT_MAGAZIN_LIMIT = 1;
const int LEFT_PUSHER_LIMIT = 2;
const int RIGHT_PUSHER_LIMIT = 3;
const int LOADER_LIMIT = 4;
const int ELEVATION_LIMIT = 5;
// const int SHOOTER_LIMIT = 6;

void init();

template <typename... Args>
void logInfo(std::string msg, Args... args) {
  RCLCPP_INFO(node->get_logger(), msg.c_str(), args...);
}

template <typename... Args>
void logError(std::string msg, Args... args) {
  RCLCPP_ERROR(node->get_logger(), msg.c_str(), args...);
}

const float magazineMin = -6.0f;
float calcMagazinePos(int magazineNum);

float calcAngle2ElevPos(float angle);

float calcLoaderPos(float elevAngle);