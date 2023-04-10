#include "../include/shot_ctrl/node.hpp"
#include "../include/shot_ctrl/shot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

FSM_INITIAL_STATE(ShotState, Start)
Context context;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  init();
  ShotState::start();

  auto timer = node->create_wall_timer(
      10ms, [=]() { ShotState::dispatch(UpdateEvent()); });

  auto eventSub = node->create_subscription<std_msgs::msg::String>(
      "shot_ctrl/sendEvent", 10,
      [=](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "MAGAZIN_LOADED_LEFT") {
          ShotState::dispatch(MagazinLoadedEvent{{}, true});
        } else if (msg->data == "MAGAZIN_LOADED_RIGHT") {
          ShotState::dispatch(MagazinLoadedEvent{{}, false});
        } else if (msg->data == "MAIN_SHOT") {
          ShotState::dispatch(MainShotEvent());
        }
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}