#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <rogilink2_interfaces/msg/frame.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "md_lib/md2022.hpp"
#include "md_lib/odrive.hpp"

using namespace std::chrono_literals;

class JoyServer : public rclcpp::Node {
 public:
  JoyServer()
      : Node("er_joy_server"),
        magazine(this, "magazine"),
        chamber(this, "chamber"),
        hammer(this, "hammer") {
    RCLCPP_INFO(this->get_logger(), "joy_server is started");
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&JoyServer::joy_callback, this, std::placeholders::_1));
    rogilink2_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>(
        "/rogilink/send", 10);
  }

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  // publisher rogilink2
  rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr rogilink2_pub;

  void init() {
    magazine.init();
    magazine.setMode(Md::Mode::Voltage);
    magazine.setVoltage(0.0);

    chamber.init();
    chamber.setMode(Md::Mode::Voltage);
    chamber.setVoltage(0.0);

    hammer.init();
    hammer.setMode(Md::Mode::Voltage);
    hammer.setVoltage(0.0);

    // shooter.init();
    // shooter.setMode(Md::Mode::Velocity,
    //                 ODriveEnum::InputMode::INPUT_MODE_VEL_RAMP);
    // shooter.setVelocity(0.0);
  }

 private:
  sensor_msgs::msg::Joy prev_joy;

  bool is_initialized = false;

  // motors
  MD2022 magazine;
  MD2022 chamber;
  MD2022 hammer;
  // ODrive shooter;

  // logicool
  enum class button {
    X = 0,
    A = 1,
    B = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    LT = 6,
    RT = 7,
    BACK = 8,
    START = 9,
    L3 = 10,
    R3 = 11,
  };

  enum class axis {
    LX = 0,
    LY = 1,
    RX = 2,
    RY = 3,
    TX = 4,
    TY = 5,
  };

  // ds4
  // enum class button {
  //   X = 1,
  //   A = 0,
  //   B = 2,
  //   Y = 3,
  //   LB = 4,
  //   RB = 5,
  //   LT = 6,
  //   RT = 7,
  //   BACK = 8,
  //   START = 9,
  //   L3 = 10,
  //   R3 = 11,
  // };

  // enum class axis {
  //   LX = 0,
  //   LY = 1,
  //   RX = 2,
  //   RY = 3,
  //   TX = 4,
  //   TY = 5,
  // };

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "joy_callback is called");
    if (!is_initialized) {
      prev_joy = *msg;
      is_initialized = true;
      return;
    }

    if (msg->buttons != prev_joy.buttons) {
      // rogilink2は直接いじる必要ない限りいらない（MDlibを使用）
      // rogilink2_interfaces::msg::Frame frame;

      // detect button state change
      if (msg->buttons[static_cast<int>(button::A)] !=
          prev_joy.buttons[static_cast<int>(button::A)]) {
        if (msg->buttons[static_cast<int>(button::A)] == 1) {
          // button A is pressed
          RCLCPP_INFO(this->get_logger(), "A is pressed");
          hammer.setVoltage(1.0);
        } else {
          // button A is released
          RCLCPP_INFO(this->get_logger(), "A is released");
          hammer.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::B)] !=
                 prev_joy.buttons[static_cast<int>(button::B)]) {
        if (msg->buttons[static_cast<int>(button::B)] == 1) {
          // button B is pressed
          RCLCPP_INFO(this->get_logger(), "B is pressed");
          magazine.setVoltage(0.5);
          std::this_thread::sleep_for(0.5s);
        } else {
          // button B is released
          RCLCPP_INFO(this->get_logger(), "B is released");
          magazine.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::X)] !=
                 prev_joy.buttons[static_cast<int>(button::X)]) {
        if (msg->buttons[static_cast<int>(button::X)] == 1) {
          // button X is pressed
          RCLCPP_INFO(this->get_logger(), "X is pressed");
          chamber.setVoltage(-1.0);
        } else {
          // button X is released
          RCLCPP_INFO(this->get_logger(), "X is released");
          chamber.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::RT)] != 0) {
        // button RT is pressed
        RCLCPP_INFO(this->get_logger(), "RT is pressed");
        magazine.setVoltage(0.5);
        std::this_thread::sleep_for(0.5s);
      }
      prev_joy = *msg;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyServer>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
