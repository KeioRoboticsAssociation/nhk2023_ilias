#include "../include/shot_ctrl/node.hpp"

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr rogilink;
std::shared_ptr<LimitSwitch> limitSensor;
std::shared_ptr<MD2022> leftMagazine;
std::shared_ptr<MD2022> rightMagazine;
std::shared_ptr<MD2022> leftPusher;
std::shared_ptr<MD2022> rightPusher;
std::shared_ptr<MD2022> loader;
std::shared_ptr<MD2022> elevation;
std::shared_ptr<ODrive> shooter;
std::shared_ptr<Servo> servo;

void init() {
  node = rclcpp::Node::make_shared("shot_ctrl");
  rogilink = node->create_publisher<rogilink2_interfaces::msg::Frame>(
      "rogilink2/send", 10);

  leftMagazine = std::make_shared<MD2022>(node.get(), "magazineL");
  rightMagazine = std::make_shared<MD2022>(node.get(), "magazineR");
  leftPusher = std::make_shared<MD2022>(node.get(), "pusherL");
  rightPusher = std::make_shared<MD2022>(node.get(), "pusherR");
  loader = std::make_shared<MD2022>(node.get(), "loader");
  elevation = std::make_shared<MD2022>(node.get(), "elevation");
  shooter = std::make_shared<ODrive>(node.get(), "shooter");

  servo = std::make_shared<Servo>(node.get(), "servo");
  limitSensor = std::make_shared<LimitSwitch>(node.get(), "limit_sensor");

  leftMagazine->init();
  rightMagazine->init();
  leftPusher->init();
  rightPusher->init();
  loader->init();
  elevation->init();
  // shooter->init();
}

// 回転数6で約40cm動く
// 輪の直径は1.4cm
float calcMagazinePos(int magazineNum) {
  if (magazineNum > 10) magazineNum = 10;
  if (magazineNum <= 0) magazineNum = 1;
  return -magazineNum * 6.0f * 40.0f / 1.4f + 0.05f;
}

float calcAngle2ElevPos(float angle) { return angle; }

float calcLoaderPos(float elevAngle) { return 0.5 * elevAngle; }