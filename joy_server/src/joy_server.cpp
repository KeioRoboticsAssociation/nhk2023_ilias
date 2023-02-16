#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rogilink2_interfaces/msg/frame.hpp>

using namespace std::chrono_literals;

class JoyServer : public rclcpp::Node {
 public:
  JoyServer() : Node("joy_server") {
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&JoyServer::joy_callback, this, std::placeholders::_1));
    rogilink2_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>(
        "/rogilink/send", 10);
  }

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  // publisher rogilink2
  rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr rogilink2_pub;

 private:
  sensor_msgs::msg::Joy prev_joy;

  enum class button {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    BACK = 6,
    START = 7,
    HOME = 8,
    L3 = 9,
    R3 = 10,
  };

  enum class axis {
    LX = 0,
    LY = 1,
    LT = 2,
    RX = 3,
    RY = 4,
    RT = 5,
    DPAD_X = 6,
    DPAD_Y = 7,
  };

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons != prev_joy.buttons) {
      prev_joy = *msg;
      rogilink2_interfaces::msg::Frame frame;
      if(msg->buttons[static_cast<int>(button::A)]==1){
        
      }

      if(msg->buttons[static_cast<int>(button::B)]==1){

      }

      if(msg->buttons[static_cast<int>(button::X)]==1){

      }

      if(msg->buttons[static_cast<int>(button::Y)]==1){

      }

      if(msg->buttons[static_cast<int>(button::LB)]==1){

      }

      if(msg->buttons[static_cast<int>(button::RB)]==1){

      }

      if(msg->buttons[static_cast<int>(button::BACK)]==1){

      }

      if(msg->buttons[static_cast<int>(button::START)]==1){

      }

      if(msg->buttons[static_cast<int>(button::HOME)]==1){

      }

      if(msg->buttons[static_cast<int>(button::L3)]==1){

      }

      if(msg->buttons[static_cast<int>(button::R3)]==1){

      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joy_server");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
