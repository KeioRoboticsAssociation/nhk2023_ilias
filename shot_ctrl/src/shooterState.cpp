#include "../include/shot_ctrl/shooterState.hpp"

#include "../include/shot_ctrl/node.hpp"

namespace Shooter {

void Init::entry() {
  // TODO: odriveのhomingを使った初期化に変更する
  logInfo("[Shooter] Enter Init");
  auto onLimit = [&]() {
    shooter->setMode(ODrive::Idle);
    shooter->resetEncoder(0);
    shooter->setMode(ODrive::Position,
                     ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
    shooter->setPosition(0);
    transit<Origin>();
  };
  if (!limitSensor->getPinState(SHOOTER_LIMIT)) {
    limitSensor->addCallback(SHOOTER_LIMIT, onLimit);
    shooter->setMode(ODrive::Velocity);
    shooter->setVelocity(1);
  } else {
    onLimit();
  }
}

void Origin::entry() {
  logInfo("[Shooter] Enter Origin");
  shooter->setMode(ODrive::Position,
                   ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
  shooter->setPosition(0);
}

void Origin::react(ShooterUpdateEvent const &) {}
void Origin::react(ShotRequestEvent const &) { transit<Shooting>(); }
void Origin::react(ReloadRequestEvent const &) { transit<Moving2LoadPos>(); }

void Moving2LoadPos::entry() {
  logInfo("[Shooter] Enter Moving2LoadPos");
  shooter->setMode(ODrive::Position,
                   ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
  shooter->setPosition(1);
}

void Moving2LoadPos::react(ShooterUpdateEvent const &) {
  if (shooter->getPosition() >= 0.9) {
    transit<OnLoadPos>();
  }
}

void OnLoadPos::entry() {
  logInfo("[Shooter] Enter OnLoadPos");
  shooter->setMode(ODrive::Position,
                   ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
  shooter->setPosition(1);
}

void OnLoadPos::react(Return2OriginRequestEvent const &) {
  transit<Moving2Origin>();
}

void Shooting::entry() {
  logInfo("[Shooter] Enter Shooting");
  shooter->setPosition(10);
}

void Shooting::react(ShooterUpdateEvent const &) {
  if (shooter->getPosition() >= 9.9) {
    transit<Moving2Origin>();
  }
}

void Moving2Origin::entry() {
  logInfo("[Shooter] Enter Moving2Origin");
  shooter->setPosition(0);
}

void Moving2Origin::react(ShooterUpdateEvent const &) {
  if (shooter->getPosition() <= 0.1) {
    transit<Origin>();
  }
}
}  // namespace Shooter