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

  limitSensor = std::make_shared<LimitSwitch>(node.get(), "limit_sensor");

  leftMagazine->init();
  rightMagazine->init();
  leftPusher->init();
  rightPusher->init();
  loader->init();
  elevation->init();
  // shooter->init();
}
float calcMagazinePos(int magazineNum) { return magazineNum * 1 + 0; }

float calcElevationPos(float angle) { return angle; }

float calcLoaderPos(float elevAngle) { return 0.5 * elevAngle; }