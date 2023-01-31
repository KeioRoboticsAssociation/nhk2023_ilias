#ifndef _STATE_CTRL_CPP_
#define _STATE_CTRL_CPP_

#include "../include/tinyfsm/include/tinyfsm.hpp"
#include "main.hpp"
#include "rclcpp/rclcpp.hpp"

struct Joy_Flag : tinyfsm::Event {};
struct Idle_Flag : tinyfsm::Event {};
struct Foward_Flag : tinyfsm::Event {};

class Idle;
class Joy;
class Start;

class StateMachine : public tinyfsm::Fsm<StateMachine> {
 public:
  void react(tinyfsm::Event const &){};

  void react(Joy_Flag const &) { transit<Joy>(); }
  void react(Idle_Flag const &) { transit<Idle>(); }

  virtual void react(Foward_Flag const &){};

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

#endif  // _STATE_CTRL_CPP_