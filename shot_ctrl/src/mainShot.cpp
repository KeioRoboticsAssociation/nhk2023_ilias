#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void MainShot::entry(void) {
  logInfo("Enter MainShot");
  rogilink->publish(rogilink2_interfaces::msg::Frame());
}

void MainShot::react(UpdateEvent const &e) {
  logInfo("MainShot");
  transit<AfterShot>();
}
