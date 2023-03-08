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