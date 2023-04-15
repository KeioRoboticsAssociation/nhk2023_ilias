#include "../include/shot_ctrl/shot.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void LoadMagazine::entry(void) { logInfo("Enter LoadMagazine"); }

void LoadMagazine::react(UpdateEvent const &) {}