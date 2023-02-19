#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <rogilink2_interfaces/msg/frame.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "actuator_lib/servo.hpp"
#include "md_lib/md2022.hpp"
#include "md_lib/odrive.hpp"

using namespace std::chrono_literals;

class JoyServer : public rclcpp::Node {
 public:
  JoyServer()
      : Node("er_joy_server"),
        magazine(this, "magazine"),
        angle(this, "angle"),
        supply(this, "supply"),
        catapult(this, "catapult"),
        servo(this, "servo") {
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

    angle.init();
    angle.setMode(Md::Mode::Voltage);
    angle.setPosition(0.0);

    supply.init();
    supply.setMode(Md::Mode::Voltage);
    supply.setVoltage(0.0);

    catapult.init();
    catapult.setMode(Md::Mode::Position);
    catapult.setPosition(0.0);

    servo.init();
    // 0: lock 1: lock 2: supply
    servo.setPosition(0, 110);  // 50
    servo.setPosition(1, 50);   // 120
    servo.setPosition(2, 125);  // 65
  }

 private:
  sensor_msgs::msg::Joy prev_joy;

  bool is_initialized = false;

  // motors
  MD2022 magazine;
  MD2022 angle;
  MD2022 supply;
  MD2022 catapult;
  Servo servo;

  double current_angle_position = 0.0;

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
          supply.setVoltage(1.0);
        } else {
          // button A is released
          RCLCPP_INFO(this->get_logger(), "A is released");
          supply.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          supply.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          supply.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::B)] !=
                 prev_joy.buttons[static_cast<int>(button::B)]) {
        if (msg->buttons[static_cast<int>(button::B)] == 1) {
          // button B is pressed
          RCLCPP_INFO(this->get_logger(), "B is pressed");
          magazine.setVoltage(0.5);
        } else {
          // button B is released
          RCLCPP_INFO(this->get_logger(), "B is released");
          magazine.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazine.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazine.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::Y)] !=
                 prev_joy.buttons[static_cast<int>(button::Y)]) {
        if (msg->buttons[static_cast<int>(button::Y)] == 1) {
          // button Y is pressed
          RCLCPP_INFO(this->get_logger(), "Y is pressed");
          magazine.setVoltage(-0.5);
        } else {
          // button Y is released
          RCLCPP_INFO(this->get_logger(), "Y is released");
          magazine.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazine.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazine.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::RB)] !=
                 prev_joy.buttons[static_cast<int>(button::RB)]) {
        if (msg->buttons[static_cast<int>(button::RB)] == 1) {
          // button RB is pressed
          RCLCPP_INFO(this->get_logger(), "RB is pressed");
          angle.setVoltage(-0.1);
        } else {
          // button RB is released
          RCLCPP_INFO(this->get_logger(), "RB is released");
          angle.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          angle.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          angle.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::RT)] !=
                 prev_joy.buttons[static_cast<int>(button::RT)]) {
        if (msg->buttons[static_cast<int>(button::RT)] == 1) {
          // button RT is pressed
          RCLCPP_INFO(this->get_logger(), "RT is pressed");
          angle.setVoltage(0.1);
        } else {
          // button RT is released
          RCLCPP_INFO(this->get_logger(), "RT is released");
          angle.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          angle.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          angle.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::LB)] !=
                 prev_joy.buttons[static_cast<int>(button::LB)]) {
        if (msg->buttons[static_cast<int>(button::LB)] == 1) {
          // button LB is pressed
          RCLCPP_INFO(this->get_logger(), "LB is pressed");
          catapult.setPosition(-4.8);
        }
      } else if (msg->buttons[static_cast<int>(button::LT)] !=
                 prev_joy.buttons[static_cast<int>(button::LT)]) {
        if (msg->buttons[static_cast<int>(button::LT)] == 1) {
          // button LT is pressed
          RCLCPP_INFO(this->get_logger(), "LT is pressed");
          catapult.setPosition(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::X)] !=
                 prev_joy.buttons[static_cast<int>(button::X)]) {
        if (msg->buttons[static_cast<int>(button::X)] == 1) {
          // button X is pressed
          RCLCPP_INFO(this->get_logger(), "X is pressed");
          // kaihou
          catapult.setMode(Md::Mode::Idle);
          servo.setPosition(0, 50);
          servo.setPosition(1, 120);
          std::this_thread::sleep_for(0.05s);
          catapult.setMode(Md::Mode::Idle);
          servo.setPosition(0, 50);
          servo.setPosition(1, 120);
          std::this_thread::sleep_for(0.05s);
          catapult.setMode(Md::Mode::Idle);
          servo.setPosition(0, 50);
          servo.setPosition(1, 120);
        }
      } else if (msg->buttons[static_cast<int>(button::START)] !=
                 prev_joy.buttons[static_cast<int>(button::START)]) {
        if (msg->buttons[static_cast<int>(button::START)] == 1) {
          catapult.setPosition(0.0);
          catapult.setMode(Md::Mode::Position);
          catapult.setPosition(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::LB)] !=
                 prev_joy.buttons[static_cast<int>(button::LB)]) {
        if (msg->buttons[static_cast<int>(button::LB)] == 1) {
          // button LB is pressed
          RCLCPP_INFO(this->get_logger(), "LB is pressed");
          servo.setPosition(2, 125);
        }
      } else if (msg->buttons[static_cast<int>(button::LT)] !=
                 prev_joy.buttons[static_cast<int>(button::LT)]) {
        if (msg->buttons[static_cast<int>(button::LT)] == 1) {
          // button LT is pressed
          RCLCPP_INFO(this->get_logger(), "LT is pressed");
          servo.setPosition(2, 65);
        }
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
