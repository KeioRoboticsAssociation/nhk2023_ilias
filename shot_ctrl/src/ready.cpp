#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shot.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void Ready::entry(void) { logInfo("Enter Ready"); }

void Ready::react(MainShotEvent const &) {
  shot();
  context.hasShuttleRing = false;
  transit<LoadLoader>();
}