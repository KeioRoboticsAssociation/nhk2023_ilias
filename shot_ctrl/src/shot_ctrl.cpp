#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shooterState.hpp"
#include "../include/shot_ctrl/shot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
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
  bool prevAngleReqValue = 0;

  auto timer = node->create_wall_timer(10ms, [&]() {
    if (prevShotReqFailed && ShooterState::is_in_state<Shooter::Origin>()) {
      ShotState::dispatch(MainShotEvent());
      prevShotReqFailed = false;
    }
    if (prevAngleReqFailed &&
        ShotState::current_state_ptr->canChangeElevation()) {
      context.elevAngle = prevAngleReqValue;
      elevation->setPosition(calcAngle2ElevPos(prevAngleReqValue));
      prevAngleReqFailed = false;
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

  auto elevSub = node->create_subscription<std_msgs::msg::Float64>(
      "shot_ctrl/setElev", 10,
      [&](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!ShotState::current_state_ptr->canChangeElevation()) {
          prevAngleReqFailed = true;
          prevAngleReqValue = msg->data;
          return;
        }
        float angle = msg->data;
        context.elevAngle = angle;
        elevation->setPosition(calcAngle2ElevPos(angle));
        ShotState::dispatch(ChangeElevationRequestEvent{{}, angle});
      });

  auto elevPub =
      node->create_publisher<std_msgs::msg::Float64>("shot_ctrl/getElev", 10);

  auto elevTimer = node->create_wall_timer(100ms, [&]() {
    std_msgs::msg::Float64 msg;
    msg.data = context.elevAngle;
    elevPub->publish(msg);
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}