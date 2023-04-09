#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void ReloadStep1::entry(void) { logInfo("Enter ReloadStep1"); }

void ReloadStep1::react(UpdateEvent const &e) {
  logInfo("ReloadStep1");
  transit<ReloadStep2>();
}