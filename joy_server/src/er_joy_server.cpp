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
        magazineR(this, "magazineR"),
        magazineL(this, "magazineL"),
        pusherL(this, "pusherL"),
        pusherR(this, "pusherR"),
        loader(this, "loader"),
        angle(this, "angle"),
        shooter(this, "shooter"),
        servo(this, "servo") {
    RCLCPP_INFO(this->get_logger(), "joy_server is started");
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&JoyServer::joy_callback, this, std::placeholders::_1));
    rogilink2_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>(
        "rogilink2/send", 10);
  }

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  // publisher rogilink2
  rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr rogilink2_pub;

  void init() {
    magazineR.init();
    magazineR.setMode(Md::Mode::Voltage);
    magazineR.setVoltage(0.0);

    magazineL.init();
    magazineL.setMode(Md::Mode::Voltage);
    magazineL.setVoltage(0.0);

    pusherL.init();
    pusherL.setMode(Md::Mode::Position);
    pusherL.setPosition(0.0);

    pusherR.init();
    pusherR.setMode(Md::Mode::Position);
    pusherR.setPosition(0.0);

    loader.init();
    loader.setMode(Md::Mode::Position);
    loader.setPosition(0.0);

    angle.init();
    angle.setMode(Md::Mode::Position);
    angle.setPosition(0.0);

    shooter.init();
    shooter.setMode(Md::Mode::Position);
    shooter.setPosition(0.0);

    servo.init();
    servo.setPosition(0, 80);
    servo.setPosition(1, 90);
  }

 private:
  sensor_msgs::msg::Joy prev_joy;

  bool is_initialized = false;

  // motors
  MD2022 magazineR;
  MD2022 magazineL;
  MD2022 pusherL;
  MD2022 pusherR;
  MD2022 loader;
  MD2022 angle;

  ODrive shooter;

  Servo servo;

  bool servo_flip = true;
  bool R_pushed = true;
  bool L_pushed = true;
  float shooter_pos = 0;

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
      if (msg->buttons[static_cast<int>(button::X)] !=
          prev_joy.buttons[static_cast<int>(button::X)]) {
        if (msg->buttons[static_cast<int>(button::X)] == 1) {
          // button X is pressed
          RCLCPP_INFO(this->get_logger(), "X is pressed");
          if (servo_flip) {
            servo.setPosition(0, 80);

            servo.setPosition(1, 95);
          } else {
            servo.setPosition(0, 0);
            servo.setPosition(1, 180);
          }
          servo_flip = !servo_flip;
        }
      } else if (msg->buttons[static_cast<int>(button::B)] !=
                 prev_joy.buttons[static_cast<int>(button::B)]) {
        if (msg->buttons[static_cast<int>(button::B)] == 1) {
          // button LB is pressed
          RCLCPP_INFO(this->get_logger(), "B is pressed");
          shooter_pos = shooter_pos + 0.1;
          shooter.setPosition(shooter_pos);
          // RCLCPP_WARN(this->get_logger(), "shooter_pos: %d", shooter_pos);
        }
      } else if (msg->buttons[static_cast<int>(button::Y)] !=
                 prev_joy.buttons[static_cast<int>(button::Y)]) {
        if (msg->buttons[static_cast<int>(button::Y)] == 1) {
          // button B is pressed
          RCLCPP_WARN(this->get_logger(), "Y is pressed");
          shooter_pos = shooter_pos + 0.5;
          shooter.setPosition(shooter_pos);
          // RCLCPP_WARN(this->get_logger(), "shooter_pos: %d", shooter_pos);
        }
      }

      else if (msg->buttons[static_cast<int>(button::RB)] !=
               prev_joy.buttons[static_cast<int>(button::RB)]) {
        if (msg->buttons[static_cast<int>(button::RB)] == 1) {
          // button RB is pressed
          RCLCPP_INFO(this->get_logger(), "RB is pressed");
          magazineR.setVoltage(-0.5);
        } else {
          // button RB is released
          RCLCPP_INFO(this->get_logger(), "RB is released");
          magazineR.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazineR.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazineR.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::RT)] !=
                 prev_joy.buttons[static_cast<int>(button::RT)]) {
        if (msg->buttons[static_cast<int>(button::RT)] == 1) {
          // button RT is pressed
          RCLCPP_INFO(this->get_logger(), "RT is pressed");
          magazineR.setVoltage(0.5);
        } else {
          // button RT is released
          RCLCPP_INFO(this->get_logger(), "RT is released");
          magazineR.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazineR.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazineR.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::LB)] !=
                 prev_joy.buttons[static_cast<int>(button::LB)]) {
        if (msg->buttons[static_cast<int>(button::LB)] == 1) {
          // button LB is pressed
          RCLCPP_INFO(this->get_logger(), "LB is pressed");
          magazineL.setVoltage(0.5);
        } else {
          // button LB is released
          RCLCPP_INFO(this->get_logger(), "LB is released");
          magazineL.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazineL.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazineL.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::LT)] !=
                 prev_joy.buttons[static_cast<int>(button::LT)]) {
        if (msg->buttons[static_cast<int>(button::LT)] == 1) {
          // button LT is pressed
          RCLCPP_INFO(this->get_logger(), "LT is pressed");
          magazineL.setVoltage(-0.5);
        } else {
          // button LT is released
          RCLCPP_INFO(this->get_logger(), "LT is released");
          magazineL.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazineL.setVoltage(0.0);
          std::this_thread::sleep_for(0.1s);
          magazineL.setVoltage(0.0);
        }
      } else if (msg->buttons[static_cast<int>(button::BACK)] !=
                 prev_joy.buttons[static_cast<int>(button::BACK)]) {
        if (msg->buttons[static_cast<int>(button::BACK)] == 1) {
          // button LB is pressed
          RCLCPP_INFO(this->get_logger(), "BACK is pressed");
          if (L_pushed) {
            pusherL.setPosition(-1.8);
          } else {
            pusherL.setPosition(4.2);
          }
          // flip L_pushed
          L_pushed = !L_pushed;
        }
      } else if (msg->buttons[static_cast<int>(button::START)] !=
                 prev_joy.buttons[static_cast<int>(button::START)]) {
        if (msg->buttons[static_cast<int>(button::START)] == 1) {
          // button LB is pressed
          RCLCPP_INFO(this->get_logger(), "START is pressed");
          if (R_pushed) {
            pusherR.setPosition(-1.8);
          } else {
            pusherR.setPosition(4.2);
          }
          // flip R_pushed
          R_pushed = !R_pushed;
        }
      } else if (msg->buttons[static_cast<int>(button::R3)] !=
                 prev_joy.buttons[static_cast<int>(button::R3)]) {
        if (msg->buttons[static_cast<int>(button::R3)] == 1) {
          // button LB is pressed
          RCLCPP_INFO(this->get_logger(), "R3 is pressed");
          shooter.setPosition(0.0);
          shooter_pos = 0.0;
        }
      } else if (msg->buttons[static_cast<int>(button::L3)] !=
                 prev_joy.buttons[static_cast<int>(button::L3)]) {
        if (msg->buttons[static_cast<int>(button::L3)] == 1) {
          // button LB is pressed
          RCLCPP_INFO(this->get_logger(), "L3 is pressed");
          shooter.setPosition(5.0);
          shooter_pos = 5.0;
        }
      }
    }

    if (msg->axes[static_cast<int>(axis::TY)] > 0.5) {
      // TY up is pressed
      RCLCPP_INFO(this->get_logger(), "TY up is pressed");
      loader.setPosition(0.6);
      std::this_thread::sleep_for(0.1s);
      loader.setPosition(0.6);
      std::this_thread::sleep_for(0.1s);
      loader.setPosition(0.6);
    } else if (msg->axes[static_cast<int>(axis::TY)] < -0.5) {
      // TY down is pressed
      RCLCPP_INFO(this->get_logger(), "TY down is pressed");
      loader.setPosition(3.5);
      std::this_thread::sleep_for(0.1s);
      loader.setPosition(3.5);
      std::this_thread::sleep_for(0.1s);
      loader.setPosition(3.5);
    }

    prev_joy = *msg;
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
