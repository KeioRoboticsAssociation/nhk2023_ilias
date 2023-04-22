#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void LeftMagazineDown::entry() {
  logInfo("Enter LeftMagazineDown");
  leftMagazine->setPosition(magazineMin);
}

void LeftMagazineDown::react(UpdateEvent const &) {}
void LeftMagazineDown::react(MagazineLoadedEvent const &) {
  transit<LoadLeftMagazine>();
}

void RightMagazineDown::entry() {
  logInfo("Enter RightMagazineDown");
  rightMagazine->setPosition(magazineMin);
}
void RightMagazineDown::react(UpdateEvent const &) {}
void RightMagazineDown::react(MagazineLoadedEvent const &) {
  transit<LoadRightMagazine>();
}