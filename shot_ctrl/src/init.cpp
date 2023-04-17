#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void Init::entry() {
  logInfo("Enter Init");

  if (!limitSensor->getPinState(LEFT_MAGAZIN_LIMIT)) {
    limitSensor->addCallback(LEFT_MAGAZIN_LIMIT, [&]() {
      leftMagazine->setVoltage(0);
      leftMagazine->resetEncoder(0);
      leftMagazine->setMode(MD2022::Position);
      leftMagazine->setPosition(0);
    });
    leftMagazine->setMode(MD2022::Voltage);
    leftMagazine->setVelocity(0.1);
  }
  if (!limitSensor->getPinState(RIGHT_MAGAZIN_LIMIT)) {
    limitSensor->addCallback(RIGHT_MAGAZIN_LIMIT, [&]() {
      rightMagazine->setVoltage(0);
      rightMagazine->resetEncoder(0);
      rightMagazine->setMode(MD2022::Position);
      rightMagazine->setPosition(0);
    });
    rightMagazine->setMode(MD2022::Voltage);
    rightMagazine->setVelocity(0.1);
  }
  if (!limitSensor->getPinState(LEFT_PUSHER_LIMIT)) {
    limitSensor->addCallback(LEFT_PUSHER_LIMIT, [&]() {
      leftPusher->setVoltage(0);
      leftPusher->resetEncoder(0);
      leftPusher->setMode(MD2022::Position);
      leftPusher->setPosition(0);
    });
    leftPusher->setMode(MD2022::Voltage);
    leftPusher->setVelocity(0.1);
  }
  if (!limitSensor->getPinState(RIGHT_PUSHER_LIMIT)) {
    limitSensor->addCallback(RIGHT_PUSHER_LIMIT, [&]() {
      rightPusher->setVoltage(0);
      rightPusher->resetEncoder(0);
      rightPusher->setMode(MD2022::Position);
      rightPusher->setPosition(0);
    });
    rightPusher->setMode(MD2022::Voltage);
    rightPusher->setVelocity(0.1);
  }
  if (!limitSensor->getPinState(LOADER_LIMIT)) {
    limitSensor->addCallback(LOADER_LIMIT, [&]() {
      loader->setVoltage(0);
      loader->resetEncoder(0);
      loader->setMode(MD2022::Position);
      loader->setPosition(0);
    });
    loader->setMode(MD2022::Voltage);
    loader->setVelocity(0.1);
  }
  if (!limitSensor->getPinState(ELEVATION_LIMIT)) {
    limitSensor->addCallback(ELEVATION_LIMIT, [&]() {
      elevation->setVoltage(0);
      elevation->resetEncoder(0);
      elevation->setMode(MD2022::Position);
      elevation->setPosition(0);
    });
    elevation->setMode(MD2022::Voltage);
    elevation->setVelocity(0.1);
  }
}

void Init::react(UpdateEvent const &) {}

void Init::react(MagazinLoadedEvent const &e) {
  if (e.isLeft == Context::LEFT) {
    context.leftRemain = 10;
    transit<LoadLeftMagazine>();
  } else {
    context.rightRemain = 10;
    transit<LoadRightMagazine>();
  }
}