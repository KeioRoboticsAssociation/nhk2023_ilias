#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <rogilink2_interfaces/msg/frame.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "actuator_lib/servo.hpp"
#include "actuator_lib/solenoid.hpp"
#include "md_lib/md2022.hpp"
#include "md_lib/odrive.hpp"

using namespace std::chrono_literals;

class JoyServer : public rclcpp::Node {
 public:
  JoyServer()
      : Node("rr_joy_server"),
        magazine(this, "magazine"),
        loader(this, "loader"),
        frontLift(this, "front_lift", 10, 5),
        backLift(this, "back_lift", 10, 5),
        frontSolenoid(this, "front_solenoid"),
        backSolenoid(this, "back_solenoid"),
        servo(this, "servo") {
    RCLCPP_INFO(this->get_logger(), "joy_server is started");
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&JoyServer::joy_callback, this, std::placeholders::_1));
    rogilink2_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>(
        "rogilink2/send", 10);

    timer = this->create_wall_timer(
        100ms, std::bind(&JoyServer::timer_callback, this));
  }

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  // publisher rogilink2
  rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr rogilink2_pub;
  // timer
  rclcpp::TimerBase::SharedPtr timer;

  void init() {
    magazine.init();
    magazine.setMode(Md::Mode::Position);
    magazine.setPosition(0.0);

    loader.init();
    loader.setMode(Md::Mode::Voltage);
    loader.setVoltage(0.0);

    // shooter.init();
    // shooter.setMode(Md::Mode::Velocity,
    //                 ODriveEnum::InputMode::INPUT_MODE_VEL_RAMP);
    // shooter.setVelocity(0.0);
    // shooterESC化のためコメントアウト

    // frontLift.init();
    frontLift.setMode(ODrive::Idle);

    // backLift.init();
    backLift.setMode(ODrive::Idle);

    frontSolenoid.init();
    backSolenoid.init();

    servo.init();
  }

 private:
  sensor_msgs::msg::Joy prev_joy;

  bool is_initialized = false;

  bool is_front_lift_moving = false;
  bool is_back_lift_moving = false;
  bool servo_flip = false;

  // motors
  MD2022 magazine;
  MD2022 loader;
  // ODrive shooter;
  ODrive frontLift;
  ODrive backLift;
  Solenoid frontSolenoid;
  Solenoid backSolenoid;
  Servo servo;

  double current_magazine_position = 0.0;
  double ring_thickness = 0.26;

  enum LiftState {
    ORIGIN = 0,
    DOWN = 1,
    UP = 2,
    MID = 3

  } frontLiftTarget = ORIGIN,
    backLiftTarget = ORIGIN;

  LiftState nextLiftState(LiftState state) {
    if (state >= LiftState::UP) return LiftState::UP;
    return static_cast<LiftState>((static_cast<int>(state) + 1) % 3);
  }

  LiftState backLiftState(LiftState state) {
    if (state == LiftState::ORIGIN) return LiftState::ORIGIN;
    return static_cast<LiftState>((static_cast<int>(state) - 1) % 3);
  }

  float LiftPos[4] = {0.1, -1.5, -4.2, -0.03};  // harasuri, low , high

  void frontSolenoidDrive(bool state) {
    for (int i = 0; i < 4; i++) {
      frontSolenoid.drive(i, state);
      std::this_thread::sleep_for(1ms);
    }
  }

  void backSolenoidDrive(bool state) {
    for (int i = 0; i < 4; i++) {
      backSolenoid.drive(i, state);
      std::this_thread::sleep_for(1ms);
    }
  }

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
    // RCLCPP_INFO(this->get_logger(), "joy_callback is called");

    if (!is_initialized) {
      prev_joy = *msg;
      is_initialized = true;
      return;
    }

    // Lift全体昇降
    if (msg->axes[static_cast<int>(axis::TX)] !=
        prev_joy.axes[static_cast<int>(axis::TX)]) {
      RCLCPP_INFO(this->get_logger(), "TX is changed");
      if (frontLiftTarget != backLiftTarget) return;
      // if (is_front_lift_moving || is_back_lift_moving) {
      //   RCLCPP_INFO(this->get_logger(), "Lift is moving");
      //   return;
      // }

      for (int i = 0; i < 5; i++) {
        frontLift.setAxisState(
            ODriveEnum::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        backLift.setAxisState(
            ODriveEnum::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        std::this_thread::sleep_for(10ms);
      }
      std::this_thread::sleep_for(1s);
      for (int i = 0; i < 5; i++) {
        frontLift.setPosition(LiftPos[frontLiftTarget]);
        std::this_thread::sleep_for(1ms);
        backLift.setPosition(LiftPos[backLiftTarget]);
        std::this_thread::sleep_for(10ms);
      }
      if (msg->axes[static_cast<int>(axis::TX)] > 0) {
        frontLiftTarget = backLiftTarget = nextLiftState(frontLiftTarget);

        // frontSolenoidDrive(1);
        // backSolenoidDrive(1);
      } else if (msg->axes[static_cast<int>(axis::TX)] < 0) {
        frontLiftTarget = backLiftTarget = backLiftState(backLiftTarget);
        // frontSolenoidDrive(0);
        // backSolenoidDrive(0);
      }
      RCLCPP_INFO(this->get_logger(), "frontLiftTarget: %f",
                  LiftPos[frontLiftTarget]);
      RCLCPP_INFO(this->get_logger(), "backLiftTarget: %f",
                  LiftPos[backLiftTarget]);

      std::this_thread::sleep_for(1s);
      for (int i = 0; i < 5; i++) {
        frontSolenoidDrive(1);
        std::this_thread::sleep_for(1ms);
        backSolenoidDrive(1);
        std::this_thread::sleep_for(1ms);
      }
      std::this_thread::sleep_for(1s);

      for (int i = 0; i < 5; i++) {
        frontLift.setPosition(LiftPos[frontLiftTarget]);
        std::this_thread::sleep_for(1ms);
        backLift.setPosition(LiftPos[backLiftTarget]);
        std::this_thread::sleep_for(10ms);
      }
      is_front_lift_moving = true;
      is_back_lift_moving = true;
    }

    // Lift個別で上限まで上げ
    if (msg->axes[static_cast<int>(axis::TY)] !=
        prev_joy.axes[static_cast<int>(axis::TY)]) {
      // solenoidを開ける
      RCLCPP_INFO(this->get_logger(), "TY is changed");
      // if (is_front_lift_moving || is_back_lift_moving) return;

      if (msg->axes[static_cast<int>(axis::TY)] < -0.5) {
        for (int i = 0; i < 5; i++) {
          backLift.setAxisState(
              ODriveEnum::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
          std::this_thread::sleep_for(1ms);
        }
        std::this_thread::sleep_for(1s);
        backLift.setPosition(LiftPos[backLiftTarget]);
        std::this_thread::sleep_for(1s);
        for (int i = 0; i < 4; i++) {
          backSolenoidDrive(1);
          std::this_thread::sleep_for(1ms);
        }
        backLiftTarget = MID;
        // RCLCPP_INFO(this->get_logger(), "backLiftTarget: %f",
        //             LiftPos[backLiftTarget]);
        std::this_thread::sleep_for(1s);
        backLift.setPosition(LiftPos[backLiftTarget]);
        is_back_lift_moving = true;
      } else if (msg->axes[static_cast<int>(axis::TY)] > 0.5) {
        for (int i = 0; i < 5; i++) {
          frontLift.setAxisState(
              ODriveEnum::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
          std::this_thread::sleep_for(1ms);
        }
        std::this_thread::sleep_for(1s);
        for (int i = 0; i < 5; i++) {
          frontLift.setPosition(LiftPos[frontLiftTarget]);
          backLift.setPosition(LiftPos[backLiftTarget]);
          std::this_thread::sleep_for(1ms);
        }
        std::this_thread::sleep_for(1s);
        for (int i = 0; i < 3; i++) {
          frontSolenoidDrive(1);
          backSolenoidDrive(1);
          std::this_thread::sleep_for(10ms);
        }

        frontLiftTarget = MID;
        RCLCPP_INFO(this->get_logger(), "frontLiftTarget: %f",
                    LiftPos[frontLiftTarget]);
        std::this_thread::sleep_for(1s);
        frontLift.setPosition(LiftPos[frontLiftTarget]);
        is_front_lift_moving = true;
      }
    }

    ////////////////////////////////////////////////
    // ------------------flip flop------------------
    ////////////////////////////////////////////////
    if (msg->buttons != prev_joy.buttons) {
      // rogilink2は直接いじる必要ない限りいらない（MDlibを使用）
      // rogilink2_interfaces::msg::Frame frame;

      // loader
      if (msg->buttons[static_cast<int>(button::A)] !=
          prev_joy.buttons[static_cast<int>(button::A)]) {
        if (msg->buttons[static_cast<int>(button::A)] == 1) {
          // button A is pressed
          RCLCPP_INFO(this->get_logger(), "A is pressed");
          loader.setVoltage(1.0);
        } else {
          // button A is released
          RCLCPP_INFO(this->get_logger(), "A is released");
          loader.setVoltage(0.0);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          loader.setVoltage(0.0);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          loader.setVoltage(0.0);
        }

      }

      // blocking servo
      else if (msg->buttons[static_cast<int>(button::X)] !=
               prev_joy.buttons[static_cast<int>(button::X)]) {
        if (msg->buttons[static_cast<int>(button::X)] == 1) {
          // button X is pressed
          RCLCPP_INFO(this->get_logger(), "X is pressed");
          if (servo_flip) {
            servo.setPosition(0, 0.0);
          } else {
            servo.setPosition(0, 60);
          }
          servo_flip = !servo_flip;
        }
      }

      // shooter
      else if (msg->buttons[static_cast<int>(button::RT)] !=
               prev_joy.buttons[static_cast<int>(button::RT)]) {
        if (msg->buttons[static_cast<int>(button::RT)] == 1) {
          // button RT is pressed
          RCLCPP_INFO(this->get_logger(), "RT is pressed");
          servo.setPosition(1, 90);
        } else {
          // button RT is released
          RCLCPP_INFO(this->get_logger(), "RT is released");
          servo.setPosition(1, 0);
        }
      }

      // magazine up
      else if (msg->buttons[static_cast<int>(button::Y)] !=
               prev_joy.buttons[static_cast<int>(button::Y)]) {
        if (msg->buttons[static_cast<int>(button::Y)] == 1) {
          // button Y is pressed
          RCLCPP_INFO(this->get_logger(), "Y is pressed");
          current_magazine_position += ring_thickness;
          magazine.setPosition(current_magazine_position);
        }
      }

      // magazin down
      else if (msg->buttons[static_cast<int>(button::B)] !=
               prev_joy.buttons[static_cast<int>(button::B)]) {
        if (msg->buttons[static_cast<int>(button::B)] == 1) {
          // button B is pressed
          RCLCPP_INFO(this->get_logger(), "B is pressed");
          current_magazine_position -= ring_thickness;
          magazine.setPosition(current_magazine_position);
        }
      }

      // solenoid in
      else if (msg->buttons[static_cast<int>(button::START)] !=
               prev_joy.buttons[static_cast<int>(button::START)]) {
        if (msg->buttons[static_cast<int>(button::START)] == 1) {
          // button START is pressed
          RCLCPP_INFO(this->get_logger(), "START is pressed");
          frontSolenoidDrive(1);
          backSolenoidDrive(1);
        }
      }

      // solenoid out
      else if (msg->buttons[static_cast<int>(button::BACK)] !=
               prev_joy.buttons[static_cast<int>(button::BACK)]) {
        if (msg->buttons[static_cast<int>(button::BACK)] == 1) {
          // button BACK is pressed
          RCLCPP_INFO(this->get_logger(), "BACK is pressed");
          frontSolenoidDrive(0);
          backSolenoidDrive(0);
          backSolenoidDrive(0);
          std::this_thread::sleep_for(10ms);
          backSolenoidDrive(0);
          std::this_thread::sleep_for(10ms);
          backSolenoidDrive(0);
          if (backLiftTarget != MID) {
            std::this_thread::sleep_for(1s);
            for (int i = 0; i < 5; i++) {
              backLift.setMode(ODrive::Idle);
              std::this_thread::sleep_for(1ms);
            }
          }
        }
      }

      prev_joy = *msg;
    }
  }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "front: %f, back: %f",
                frontLift.getPosition(), backLift.getPosition());

    if (abs(frontLift.getPosition() - LiftPos[frontLiftTarget]) < 0.01 &&
        is_front_lift_moving) {
      // solenoidを出す
      RCLCPP_INFO(this->get_logger(), "front lift is arrived");
      for (int i = 0; i < 5; i++) {
        frontSolenoidDrive(0);
        std::this_thread::sleep_for(1ms);
      }
      if (frontLiftTarget != MID) {
        std::this_thread::sleep_for(1s);
        for (int i = 0; i < 5; i++) {
          frontLift.setMode(ODrive::Idle);
          std::this_thread::sleep_for(1ms);
        }
      }

      is_front_lift_moving = false;
    }
    if (abs(backLift.getPosition() - LiftPos[backLiftTarget]) < 0.01 &&
        is_back_lift_moving) {
      RCLCPP_INFO(this->get_logger(), "back lift is arrived");
      backSolenoidDrive(0);
      std::this_thread::sleep_for(10ms);
      backSolenoidDrive(0);
      std::this_thread::sleep_for(10ms);
      backSolenoidDrive(0);
      if (backLiftTarget != MID) {
        std::this_thread::sleep_for(1s);
        for (int i = 0; i < 5; i++) {
          backLift.setMode(ODrive::Idle);
          std::this_thread::sleep_for(1ms);
        }
      }

      is_back_lift_moving = false;
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
