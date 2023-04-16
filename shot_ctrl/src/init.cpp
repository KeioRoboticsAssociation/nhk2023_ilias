#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void Init::entry() {
  logInfo("Enter Init");

  if (!limitSensor->getPinState(MAGAZIN_LIMIT)) {
    limitSensor->addCallback(MAGAZIN_LIMIT, [&]() {
      magazin->setVoltage(0);
      magazin->resetEncoder(0);
      magazin->setMode(MD2022::Position);
      magazin->setPosition(0);
    });
    magazin->setMode(MD2022::Voltage);
    magazin->setVelocity(0.1);
  }
  if (!limitSensor->getPinState(PUSHER_LIMIT)) {
    limitSensor->addCallback(PUSHER_LIMIT, [&]() {
      pusher->setVoltage(0);
      pusher->resetEncoder(0);
      pusher->setMode(MD2022::Position);
      pusher->setPosition(0);
    });
    pusher->setMode(MD2022::Voltage);
    pusher->setVelocity(0.1);
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
  } else {
    context.rightRemain = 10;
  }
  transit<LoadMagazine>();
}