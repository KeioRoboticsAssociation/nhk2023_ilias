#pragma once

#include "../include/shot_ctrl/shot_state.hpp"

void Start::entry(void) {
  context.leftRemain = 0;
  context.rightRemain = 0;
}

void Start::react(MagazinLoadedEvent const& event) {
  if (event.isLeft) {
    context.leftRemain = 10;
    context.usingMagazin = Context::USING_MAGAZIN::LEFT;
  } else {
    context.rightRemain = 10;
    context.usingMagazin = Context::USING_MAGAZIN::RIGHT;
  }
  transit<Loading>();
}