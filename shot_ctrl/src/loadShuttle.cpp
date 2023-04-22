#include "../include/shot_ctrl/shooterState.hpp"
#include "../include/shot_ctrl/shot_state.hpp"

void LoadShuttle::entry() {
  isInitialized = false;
  if (ShooterState::is_in_state<Shooter::Origin>()) {
    ShooterState::dispatch(ReloadRequestEvent());
    leftMagazine->setPosition(calcMagazinePos(context.leftRemain));
    rightMagazine->setPosition(calcMagazinePos(context.rightRemain));
    isInitialized = true;
  }

  logInfo("Enter LoadShuttle");
}

void LoadShuttle::react(UpdateEvent const &) {
  if (!isInitialized) entry();
  if (ShooterState::is_in_state<Shooter::OnLoadPos>()) {
    transit<UpLoader>();
  }
}