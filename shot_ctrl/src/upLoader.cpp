#include "../include/shot_ctrl/shooterState.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void UpLoader::entry() {
  isInitialized = false;
  if (!ShooterState::is_in_state<Shooter::OnLoadPos>()) {
    loader->setPosition(0);
    isInitialized = true;
  }
  logInfo("Enter UpLoader");
}

void UpLoader::react(UpdateEvent const &) {
  if (!isInitialized) entry();
  // TODO: 射角からローダーが射出しても良い位置か計算して、以下のifを判断する
  if (loader->getPosition() <= 5) {
    ShooterState::dispatch(Return2OriginRequestEvent());
    context.hasShuttleRing = true;
  }
  if (loader->getPosition() >= 0.1) {
    transit<LoadLoader>();
  }
}

void UpLoader::react(MainShotEvent const &) {
  if (ShooterState::is_in_state<Shooter::Origin>() && context.hasShuttleRing) {
    ShooterState::dispatch(ShotRequestEvent());
  }
}