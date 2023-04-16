#pragma once

#include "node.hpp"
#include "tinyfsm.hpp"

struct ShotRequestEvent : tinyfsm::Event {};
struct ReloadRequestEvent : tinyfsm::Event {};
struct Return2OriginRequestEvent : tinyfsm::Event {};
struct ShooterUpdateEvent : tinyfsm::Event {};

class ShooterState : public tinyfsm::Fsm<ShooterState> {
 public:
  ShooterState() {}
  void react(tinyfsm::Event const &) {}

  virtual void react(ShotRequestEvent const &) {}
  virtual void react(ShooterUpdateEvent const &) {}
  virtual void react(ReloadRequestEvent const &) {}
  virtual void react(Return2OriginRequestEvent const &) {}

  virtual void entry(void) {}
  virtual void exit(void) {}
};

namespace Shooter {

// 射出系の初期化
// 初期化が完了し次第、Originに遷移
class Init : public ShooterState {
 public:
  void entry(void) override;
};

// 原点(射出位置)の状態
// ShotRequestEventでShootingに遷移
// ReloadRequestEventでMoving2LoadPosに遷移
class Origin : public ShooterState {
 public:
  void entry(void) override;
  void react(ShooterUpdateEvent const &) override;
  void react(ShotRequestEvent const &) override;
  void react(ReloadRequestEvent const &) override;
};

// ロード位置へ移動中の状態
// 移動完了し次第OnLoadPosに遷移
class Moving2LoadPos : public ShooterState {
 public:
  void entry(void) override;
  void react(ShooterUpdateEvent const &) override;
};

// ロード位置の状態
// Return2OriginRequestEventでMoving2Originに遷移
class OnLoadPos : public ShooterState {
 public:
  void entry(void) override;
  void react(Return2OriginRequestEvent const &) override;
};

// 射出中の状態
// 射出完了し次第Moving2Originに遷移
class Shooting : public ShooterState {
 public:
  void entry(void) override;
  void react(ShooterUpdateEvent const &) override;
};

// 原点へ移動中の状態
// 移動完了し次第Originに遷移
class Moving2Origin : public ShooterState {
 public:
  void entry(void) override;
  void react(ShooterUpdateEvent const &) override;
};
}