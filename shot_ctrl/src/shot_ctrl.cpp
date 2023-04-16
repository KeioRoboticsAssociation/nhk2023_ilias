#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shooterState.hpp"
#include "../include/shot_ctrl/shot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

FSM_INITIAL_STATE(ShotState, Init)
FSM_INITIAL_STATE(ShooterState, Shooter::Init)
Context context;

// 仰角系の実装とデバッグ

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  init();
  ShooterState::start();
  ShotState::start();

  bool prevShotReqFailed = false;
  bool prevAngleReqFailed = false;

  auto timer = node->create_wall_timer(10ms, [&]() {
    if (prevShotReqFailed && ShooterState::is_in_state<Shooter::Origin>()) {
      ShotState::dispatch(MainShotEvent());
      prevShotReqFailed = false;
    }
    ShotState::dispatch(UpdateEvent());
    ShooterState::dispatch(ShooterUpdateEvent());
  });

  auto eventSub = node->create_subscription<std_msgs::msg::String>(
      "shot_ctrl/event", 10, [&](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "MAGAZIN_LOADED_LEFT") {
          ShotState::dispatch(MagazinLoadedEvent{{}, true});
        } else if (msg->data == "MAGAZIN_LOADED_RIGHT") {
          ShotState::dispatch(MagazinLoadedEvent{{}, false});
        } else if (msg->data == "MAIN_SHOT") {
          if (!ShooterState::is_in_state<Shooter::Origin>() ||
              !ShotState::current_state_ptr->canShot()) {
            prevShotReqFailed = true;
            return;
          }
          ShotState::dispatch(MainShotEvent());
        }
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}