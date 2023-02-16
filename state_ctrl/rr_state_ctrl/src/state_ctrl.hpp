#ifndef _STATE_CTRL_CPP_
#define _STATE_CTRL_CPP_

#include <string>

#include "../include/tinyfsm/include/tinyfsm.hpp"
#include "rclcpp/rclcpp.hpp"

struct Joy_Flag : tinyfsm::Event {};
struct Idle_Flag : tinyfsm::Event {};
struct Forward_Flag : tinyfsm::Event {};
struct GOD_Flag : tinyfsm::Event {};

class Idle;
class Joy;
class Start;

class StateMachine : public tinyfsm::Fsm<StateMachine> {
 public:
  StateMachine();
  void react(tinyfsm::Event const &){};

  void react(Joy_Flag const &) { transit<Joy>(); };
  void react(Idle_Flag const &) { transit<Idle>(); };
  void react(GOD_Flag const &, std::string destination) {
    if (destination == "start") {
      transit<Start>();
    } else if (destination == "idle") {
      transit<Idle>();
    } else if (destination == "joy") {
      transit<Joy>();
    }
  };

  virtual void react(Forward_Flag const &){};

  virtual void entry(void){};
  virtual void exit(void){};
};

class Idle : public StateMachine {
 public:
  void entry(void) override;
};

class Joy : public StateMachine {
 public:
  void entry(void) override;
};

class Start : public StateMachine {
 public:
  void entry(void) override;
};

using state_machine = StateMachine;

#endif  // _STATE_CTRL_CPP_