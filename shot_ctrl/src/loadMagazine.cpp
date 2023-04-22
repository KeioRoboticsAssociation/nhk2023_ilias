#include "../include/shot_ctrl/shot_state.hpp"

void LoadLeftMagazine::entry(void) {
  leftMagazine->setPosition(calcMagazinePos(10));
  logInfo("Enter LoadLeftMagazine");
}

void LoadLeftMagazine::react(UpdateEvent const &) {
  if (leftMagazine->getPosition() >= calcMagazinePos(10) - 0.1) {
    context.leftRemain = 10;
    transit<LoadLoader>();
  }
}

void LoadRightMagazine::entry(void) {
  rightMagazine->setPosition(calcMagazinePos(10));
  logInfo("Enter LoadRightMagazine");
}

void LoadRightMagazine::react(UpdateEvent const &) {
  if (rightMagazine->getPosition() >= calcMagazinePos(10) - 0.1) {
    context.rightRemain = 10;
    transit<Ready>();
  }
}