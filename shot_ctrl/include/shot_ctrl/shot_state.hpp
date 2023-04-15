#pragma once

#include "node.hpp"
#include "tinyfsm.hpp"

struct Context {
  bool hasShuttleRing = false;
  int leftRemain = 0;
  int rightRemain = 0;
  enum {
    LEFT,
    RIGHT,
    NONE,
  } usingMagazin = NONE;
};

extern Context context;

struct MainShotEvent : tinyfsm::Event {};
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

class Init : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
  void react(MagazinLoadedEvent const &) override;
};

class LoadMagazine : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
};

class Ready : public ShotState {
 public:
  void entry(void) override;
  void react(MainShotEvent const &) override;
  // void react(SubShotEvableShotent const &) override;
};

////  水平ローダーで押す
class LoadLoader : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
  void react(MainShotEvent const &) override;
};

class DownLoader : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
};

class LoadShuttle : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
};

class UpLoader : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
  void react(MainShotEvent const &) override;
};
