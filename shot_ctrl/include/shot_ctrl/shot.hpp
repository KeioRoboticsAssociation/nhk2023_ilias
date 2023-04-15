#pragma once

#include "./node.hpp"
#include "./shot_state.hpp"

void shot() {
  logInfo("shot");
  shooter->setPosition(0);
}