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
        shooter(this, "shooter"),
        frontLift(this, "front_lift"),
        backLift(this, "back_lift"),
        frontSolenoid(this, "front_solenoid"),
        backSolenoid(this, "back_solenoid"),
        block_servo(this, "servo") {
    RCLCPP_INFO(this->get_logger(), "joy_server is started");
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&JoyServer::joy_callback, this, std::placeholders::_1));
    rogilink2_pub = this->create_publisher<rogilink2_interfaces::msg::Frame>(
        "rogilink/send", 10);

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

    shooter.init();
    shooter.setMode(Md::Mode::Velocity,
                    ODriveEnum::InputMode::INPUT_MODE_VEL_RAMP);
    shooter.setVelocity(0.0);

    frontLift.init();
    frontLift.setMode(ODrive::Idle);

    backLift.init();
    backLift.setMode(ODrive::Idle);

    frontSolenoid.init();
    backSolenoid.init();

    block_servo.init();
  }

 private:
  sensor_msgs::msg::Joy prev_joy;

  bool is_initialized = false;

  // motors
  MD2022 magazine;
  MD2022 loader;
  ODrive shooter;
  ODrive frontLift;
  ODrive backLift;
  Solenoid frontSolenoid;
  Solenoid backSolenoid;
  Servo block_servo;

  double current_magazine_position = 0.0;
  double ring_thickness = 0.26;

  enum RiftState {
    ORIGIN = 0,
    DOWN = 1,
    UP = 2,
  } frontRiftTarget,
      backRiftTarget;

  RiftState nextRiftState(RiftState state) {
    if (state == RiftState::UP) return RiftState::UP;
    return static_cast<RiftState>((static_cast<int>(state) + 1) % 3);
  }

  RiftState backRiftState(RiftState state) {
    if (state == RiftState::ORIGIN) return RiftState::ORIGIN;
    return static_cast<RiftState>((static_cast<int>(state) + 1) % 3);
  }

  float riftPos[3] = {0, 0.5, 1};

  void frontSolenoidDrive(bool state) {
    for (int i = 0; i < 4; i++) {
      frontSolenoid.drive(i, state);
    }
  }

  void backSolenoidDrive(bool state) {
    for (int i = 0; i < 4; i++) {
      backSolenoid.drive(i, state);
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
    RCLCPP_INFO(this->get_logger(), "joy_callback is called");
    if (!is_initialized) {
      prev_joy = *msg;
      is_initialized = true;
      return;
    }

    // Rift全体昇降
    if (msg->axes[static_cast<int>(axis::TX)] !=
        prev_joy.axes[static_cast<int>(axis::TX)]) {
      if (frontRiftTarget != backRiftTarget) return;
      // solenoidを開ける
      if (msg->axes[static_cast<int>(axis::TX)] > 0.5) {
        frontRiftTarget = backRiftTarget = nextRiftState(frontRiftTarget);

      } else if (msg->axes[static_cast<int>(axis::TX)] < -0.5) {
        frontRiftTarget = backRiftTarget = backRiftState(backRiftTarget);
      }
      frontLift.setMode(ODrive::Position,
                        ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
      backLift.setMode(ODrive::Position,
                       ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
      frontSolenoidDrive(1);
      backSolenoidDrive(1);
      std::this_thread::sleep_for(100ms);
      frontLift.setPosition(riftPos[frontRiftTarget]);
      backLift.setPosition(riftPos[backRiftTarget]);
    }

    // Rift個別で上限まで上げ
    if (msg->axes[static_cast<int>(axis::TY)] !=
        prev_joy.axes[static_cast<int>(axis::TY)]) {
      if (msg->axes[static_cast<int>(axis::TY)] > 0.5) {
        backRiftTarget = ORIGIN;
        backLift.setMode(ODrive::Position,
                         ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
        backSolenoidDrive(1);
        std::this_thread::sleep_for(100ms);
        backLift.setPosition(riftPos[backRiftTarget]);
      } else if (msg->axes[static_cast<int>(axis::TY)] < -0.5) {
        frontRiftTarget = ORIGIN;
        frontLift.setMode(ODrive::Position,
                          ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
        frontSolenoidDrive(1);
        std::this_thread::sleep_for(100ms);
        frontLift.setPosition(riftPos[frontRiftTarget]);
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
          loader.setVoltage(-1.0);
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
          block_servo.setPosition(0, 0.0);
        } else {
          // button X is released
          RCLCPP_INFO(this->get_logger(), "X is released");
          block_servo.setPosition(0, 90.0);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          block_servo.setPosition(0, 90.0);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          block_servo.setPosition(0, 90.0);
        }
      }

      // shooter
      else if (msg->buttons[static_cast<int>(button::RT)] !=
               prev_joy.buttons[static_cast<int>(button::RT)]) {
        if (msg->buttons[static_cast<int>(button::RT)] == 1) {
          // button RT is pressed
          RCLCPP_INFO(this->get_logger(), "RT is pressed");
          shooter.setVelocity(-55.0);
        } else {
          // button RT is released
          RCLCPP_INFO(this->get_logger(), "RT is released");
          shooter.setVelocity(0.0);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          shooter.setVelocity(0.0);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          shooter.setVelocity(0.0);
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

      prev_joy = *msg;
    }
  }
  void timer_callback() {
    if (abs(frontLift.getPosition() - riftPos[frontRiftTarget]) < 0.05) {
      // solenoidを出す
      frontSolenoidDrive(0);
      std::this_thread::sleep_for(100ms);
      frontLift.setMode(ODrive::Idle);
    }
    if (abs(backLift.getPosition() - riftPos[backRiftTarget]) < 0.05) {
      backSolenoidDrive(0);
      std::this_thread::sleep_for(100ms);
      backLift.setMode(ODrive::Idle);
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
