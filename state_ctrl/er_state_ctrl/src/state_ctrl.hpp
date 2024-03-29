#ifndef _STATE_CTRL_CPP_
#define _STATE_CTRL_CPP_

#include <string>

#include "../include/tinyfsm/include/tinyfsm.hpp"
#include "rclcpp/rclcpp.hpp"

struct Manual_Flag : tinyfsm::Event {};
struct Idle_Flag : tinyfsm::Event {};
struct Forward_Flag : tinyfsm::Event {};
struct Start_Flag : tinyfsm::Event {};
struct GOD_Flag : tinyfsm::Event {};

struct Update_Flag : tinyfsm::Event {};
struct Shot_Flag : tinyfsm::Event {};

class Start;
class Idle;
class Manual;

class StateMachine : public tinyfsm::Fsm<StateMachine> {
 public:
  StateMachine();
  void react(tinyfsm::Event const &){};

  void react(Manual_Flag const &) { transit<Manual>(); };
  void react(Idle_Flag const &) { transit<Idle>(); };
  void react(Start_Flag const &) { transit<Start>(); };
  void react(GOD_Flag const &, std::string destination) {
    if (destination == "start") {
      transit<Start>();
    } else if (destination == "idle") {
      transit<Idle>();
    } else if (destination == "manual") {
      transit<Manual>();
    }
  };

  virtual void react(Update_Flag const &){};

  virtual void react(Forward_Flag const &){};
  virtual void react(Shot_Flag const &){};

  virtual void entry(void){};
  virtual void exit(void){};
};

class Idle : public StateMachine {
 public:
  void entry(void) override;
};

class Manual : public StateMachine {
 public:
  void entry(void) override;
};

class Start : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &flag) override;
};

class PickupLeft : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &flag) override;
  void react(Update_Flag const &flag) override;
  void exit() override;
};

class PickupRight : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &flag) override;
  void react(Update_Flag const &flag) override;
  void exit() override;
};

class PreShot : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &flag) override;
  void react(Update_Flag const &flag) override;
};

class Shot : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &flag) override;
  void react(Update_Flag const &flag) override;
};

using state_machine = StateMachine;

#endif  // _STATE_CTRL_CPP_