#pragma once

#include "node.hpp"
#include "tinyfsm.hpp"

struct Context {
  int leftRemain;
  int rightRemain;
  enum {
    LEFT,
    RIGHT,
    NONE,
  } usingMagazin;
};

extern Context context;

struct MainShotEvent : tinyfsm::Event {};
struct SubShotEvent : tinyfsm::Event {};
struct MagazinLoadedEvent : tinyfsm::Event {
  bool isLeft;
};
struct UpdateEvent : tinyfsm::Event {};

class ShotState : public tinyfsm::Fsm<ShotState> {
 public:
  ShotState() {}
  void react(tinyfsm::Event const &) {}

  virtual void react(MainShotEvent const &) { logError("Unable to Shot"); }
  virtual void react(MagazinLoadedEvent const &) {}
  virtual void react(UpdateEvent const &) {}

  virtual void entry(void) {}
  virtual void exit(void) {}
};

class Start : public ShotState {
 public:
  void entry(void) override;
  void react(MagazinLoadedEvent const &) override;
};

class Loading : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
};

class MainShot : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
};

class Ready : public ShotState {
 public:
  void entry(void) override;
  void react(MainShotEvent const &) override;
  // void react(SubShotEvent const &) override;
};

// 水平ローダーで押す
class ReloadStep1 : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
};

class ReloadStep2 : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
};

class AfterShot : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
};
