#include "../include/shot_ctrl/shooterState.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void DownLoader::entry() {
  isInitialized = false;
  if (ShooterState::is_in_state<Shooter::Origin>()) {
    loader->setPosition(calcLoaderPos(context.elevAngle));
    leftPusher->setPosition(0);
    rightPusher->setPosition(0);
    isInitialized = true;
  }
  logInfo("Enter DownLoader");
}

void DownLoader::react(UpdateEvent const &) {
  if (!isInitialized) entry();
  if (loader->getPosition() >= calcLoaderPos(context.elevAngle) + 0.1) {
    transit<LoadShuttle>();
  }
}
