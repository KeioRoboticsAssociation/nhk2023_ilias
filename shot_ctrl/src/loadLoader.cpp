#include "../include/shot_ctrl/shooterState.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

const float pusherLoadPosition = 5;

void LoadLoader::entry() {
  logInfo("Enter LoadLoader");
  isInitialized = false;
  // TODO: 左右マガジンの残弾及びusingMagazinから動かすプッシャーを選択し、
  // context.usingMagazinを設定する
  loader->setPosition(0);
  if (loader->getPosition() <= 0.1) {
    pusher->setPosition(pusherLoadPosition);
  }
}

void LoadLoader::react(UpdateEvent const &) {
  if (!isInitialized) entry();
  if (pusher->getPosition() >= pusherLoadPosition - 0.1) {
    context.leftRemain--;
    if (context.hasShuttleRing) {
      transit<Ready>();
    } else {
      transit<DownLoader>();
    }
  }
}

void LoadLoader::react(MainShotEvent const &) {
  if (ShooterState::is_in_state<Shooter::Origin>() && context.hasShuttleRing) {
    ShooterState::dispatch(ShotRequestEvent());
  }
}

void LoadLoader::exit() {}
