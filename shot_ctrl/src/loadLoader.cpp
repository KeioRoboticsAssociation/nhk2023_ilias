#include "../include/shot_ctrl/shot.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

const float pusherLoadPosition = 5;

void LoadLoader::entry() {
  logInfo("Enter LoadLoader");
  pusher->setPosition(pusherLoadPosition);
}

void LoadLoader::react(UpdateEvent const &) {
  if (pusher->getPosition() >= pusherLoadPosition - 0.1) {
    transit<DownLoader>();
  }
}