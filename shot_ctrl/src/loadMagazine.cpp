#include "../include/shot_ctrl/shot_state.hpp"

void LoadMagazine::entry(void) {
  magazin->setPosition(calcMagazinePos(10));
  logInfo("Enter LoadMagazine");
}

void LoadMagazine::react(UpdateEvent const &) {
  if (magazin->getPosition() >= calcMagazinePos(10) - 0.1) {
    transit<LoadLoader>();
  }
}