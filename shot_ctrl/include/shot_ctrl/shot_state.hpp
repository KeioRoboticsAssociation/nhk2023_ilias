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
  } usingMagazine = NONE;
  float elevAngle = 45;
};

extern Context context;

struct MainShotEvent : tinyfsm::Event {};
struct MagazinLoadedEvent : tinyfsm::Event {
  bool isLeft;
};
struct ChangeElevationRequestEvent : tinyfsm::Event {
  float angle;
};
struct UpdateEvent : tinyfsm::Event {};

class ShotState : public tinyfsm::Fsm<ShotState> {
 public:
  ShotState() {}
  void react(tinyfsm::Event const &) {}

  virtual void react(MainShotEvent const &) { logError("Unable to Shot"); }
  virtual void react(MagazinLoadedEvent const &) {}
  // virtual void react(ChangeElevationRequestEvent const &e) {}
  virtual void react(UpdateEvent const &) {}

  virtual void entry(void) {}
  virtual void exit(void) {}

  virtual bool canShot() { return false; }
  virtual bool canChangeElevation() { return false; }
};

// 装填系の初期化
// MagazineLoadedEventでLoadMagazinに遷移
class Init : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
  void react(MagazinLoadedEvent const &) override;
};

// マガジンを10本装填位置に移動
// 移動完了後、LoadLoaderに遷移
class LoadLeftMagazine : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
};

class LoadRightMagazine : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
};

// シャトルに装填完了、発射待機のみ
// hasShuttleRingがfalseの場合、DownLoaderに遷移
class Ready : public ShotState {
 public:
  void entry(void) override;
  void react(MainShotEvent const &) override;
  bool canShot() override { return true; }
  void react(MagazinLoadedEvent const &) override;
  bool canChangeElevation() override { return true; }
  // void react(SubShotEvableShotent const &) override;
};

// pusherでローダーに輪を装填
// pusherが移動完了したら、context.xxRemainを減らし、
// hasShuttleRingがtrueならReadyに遷移
// hasShuttleRingがfalseならDownLoaderに遷移
class LoadLoader : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
  void react(MainShotEvent const &) override;
  bool canShot() override { return context.hasShuttleRing; }
  bool canChangeElevation() override { return true; }
  void exit() override;

 private:
  bool isInitialized = false;
};

// ローダーを下げる
// ローダーが下がりきったら、LoadShuttleに遷移
class DownLoader : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;

 private:
  bool isInitialized = false;
};

// シャトルを動かして輪を取る。ついでにマガジンも上げる。
// shooterStateがOnLoadPosになったら、UpLoaderに遷移
class LoadShuttle : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;

 private:
  bool isInitialized = false;
};

// ローダーを上げる。ある程度上がったらシャトルを原点に戻す
// ローダーが上がりきったら、hasShuttleRingをtrueにして、LoadLoaderに遷移
class UpLoader : public ShotState {
 public:
  void entry(void) override;
  void react(UpdateEvent const &) override;
  void react(MainShotEvent const &) override;
  bool canShot() override { return context.hasShuttleRing; }

 private:
  bool isInitialized = false;
};
