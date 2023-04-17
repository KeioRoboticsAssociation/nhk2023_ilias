#include "../include/shot_ctrl/shooterState.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

const float pusherLoadPosition = 5;

void LoadLoader::entry() {
  logInfo("Enter LoadLoader");
  isInitialized = false;

  float leftMagazinePos = calcMagazinePos(context.leftRemain);
  float rightMagazinePos = calcMagazinePos(context.rightRemain);

  loader->setPosition(0);
  leftMagazine->setPosition(leftMagazinePos);
  rightMagazine->setPosition(rightMagazinePos);

  if (loader->getPosition() > 0.1 ||
      abs(leftMagazine->getPosition() - leftMagazinePos > 0.1) ||
      abs(rightMagazine->getPosition() - rightMagazinePos > 0.1))
    return;

  if (context.usingMagazine == Context::NONE) {
    context.usingMagazine = Context::LEFT;
  }

  if (context.usingMagazine == Context::LEFT) {
    if (context.rightRemain > 0) {
      rightPusher->setPosition(pusherLoadPosition);
      context.usingMagazine = Context::RIGHT;
    } else if (context.leftRemain > 0) {
      leftPusher->setPosition(pusherLoadPosition);
      context.usingMagazine = Context::LEFT;
    }
  } else {
    if (context.leftRemain > 0) {
      leftPusher->setPosition(pusherLoadPosition);
      context.usingMagazine = Context::LEFT;
    } else if (context.rightRemain > 0) {
      rightPusher->setPosition(pusherLoadPosition);
      context.usingMagazine = Context::RIGHT;
    }
  }

  leftPusher->setPosition(pusherLoadPosition);

  isInitialized = true;
}

void LoadLoader::react(UpdateEvent const &) {
  if (!isInitialized) entry();
  auto goNext = [&]() {
    if (context.hasShuttleRing) {
      transit<Ready>();
    } else {
      transit<DownLoader>();
    }
  };

  bool isFinished;
  if (context.usingMagazine == Context::LEFT) {
    if (leftPusher->getPosition() < pusherLoadPosition - 0.1) return;
    leftPusher->setPosition(0);
    context.leftRemain--;
    goNext();
  } else {
    if (rightPusher->getPosition() < pusherLoadPosition - 0.1) return;
    rightPusher->setPosition(0);
    context.rightRemain--;
    goNext();
  }
}

void LoadLoader::react(MainShotEvent const &) {
  if (ShooterState::is_in_state<Shooter::Origin>() && context.hasShuttleRing) {
    ShooterState::dispatch(ShotRequestEvent());
  }
}

void LoadLoader::exit() {}
