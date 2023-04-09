#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void ReloadStep2::entry(void) { logInfo("Enter ReloadStep2"); }

void ReloadStep2::react(UpdateEvent const &e) {
  logInfo("ReloadStep2");
  transit<Ready>();
}