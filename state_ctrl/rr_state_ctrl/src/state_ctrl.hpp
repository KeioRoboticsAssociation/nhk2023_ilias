#ifndef _STATE_CTRL_CPP_
#define _STATE_CTRL_CPP_

#include <string>

#include "../include/tinyfsm/include/tinyfsm.hpp"
#include "rclcpp/rclcpp.hpp"

struct Manual_Flag : tinyfsm::Event {};
struct Idle_Flag : tinyfsm::Event {};
struct Forward_Flag : tinyfsm::Event {};

// prepared GOD_Flag since only static function can be used as callback
struct GOD_Flag : tinyfsm::Event {};

class Idle;
class Manual;
class Start;
class Restart;
class Hill_Bottom;
class Hill_Top;
class Angkor;
class Angkor_Center;
class Type2_Attack;
class Pole_Block;
class Last_Attack;
class End;

class StateMachine : public tinyfsm::Fsm<StateMachine> {
 public:
  StateMachine();
  void react(tinyfsm::Event const &){};

  void react(Manual_Flag const &) { transit<Manual>(); };
  void react(Idle_Flag const &) { transit<Idle>(); };
  void react(GOD_Flag const &, std::string destination) {
    if (destination == "START") {
      transit<Start>();
    } else if (destination == "RESTART") {
      transit<Restart>();
    } else if (destination == "HILL_BOTTOM") {
      transit<Hill_Bottom>();
    } else if (destination == "HILL_TOP") {
      transit<Hill_Top>();
    } else if (destination == "ANGKOR") {
      transit<Angkor>();
    } else if (destination == "ANGKOR_CENTER") {
      transit<Angkor_Center>();
    } else if (destination == "TYPE2_ATTACK") {
      transit<Type2_Attack>();
    } else if (destination == "POLE_BLOCK") {
      transit<Pole_Block>();
    } else if (destination == "LAST_ATTACK") {
      transit<Last_Attack>();
    } else if (destination == "END") {
      transit<End>();
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

class Manual : public StateMachine {
 public:
  void entry(void) override;
};

class Start : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &) override { transit<Hill_Bottom>(); };
};

class Restart : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &) override { transit<Hill_Bottom>(); };
};

class Hill_Bottom : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &) override { transit<Hill_Top>(); };
};

class Hill_Top : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &) override { transit<Angkor>(); };
};

class Angkor : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &) override { transit<Angkor_Center>(); };
};

class Angkor_Center : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &) override { transit<Type2_Attack>(); };
};

class Type2_Attack : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &) override { transit<Pole_Block>(); };
};

class Pole_Block : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &) override { transit<Last_Attack>(); };
};

class Last_Attack : public StateMachine {
 public:
  void entry(void) override;
  void react(Forward_Flag const &) override { transit<End>(); };
};

class End : public StateMachine {
 public:
  void entry(void) override;
};

using state_machine = StateMachine;

#endif  // _STATE_CTRL_CPP_