#include "../include/shot_ctrl/shot.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void LoadLoader::entry() {
  logInfo("Enter LoadLoader");
  loader->setPosition(5);
}

void LoadLoader::react(UpdateEvent const &) {
  if (loader->getPosition() >= 4.9) {
    transit<DownLoader>();
  }
}