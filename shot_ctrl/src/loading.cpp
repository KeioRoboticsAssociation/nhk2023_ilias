#pragma once

#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

const float MAGAZINE_FIRST_POS = 5;

void Loading::entry() {
  logInfo("Enter Loading");
  if (context.usingMagazin == Context::USING_MAGAZIN::LEFT) {
  } else {
  }
}

void Loading::react(UpdateEvent const& event) {
  logInfo("Loading");
  transit<ReloadStep1>();
}
