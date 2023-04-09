#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void AfterShot::entry(void) { logInfo("Enter AfterShot"); }

void AfterShot::react(UpdateEvent const &e) {
  logInfo("AfterShot");
  transit<ReloadStep1>();
}