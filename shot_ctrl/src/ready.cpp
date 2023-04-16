#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shooterState.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void Ready::entry(void) { logInfo("Enter Ready"); }

void Ready::react(MainShotEvent const &) {
  ShooterState::dispatch(ShotRequestEvent());
  context.hasShuttleRing = false;
  transit<LoadLoader>();
}