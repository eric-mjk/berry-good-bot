#pragma once
#include <iostream>
#include <behaviortree_cpp_v3/bt_factory.h>  // v3 API

// --------------------------------------
// 1) FakePerception
// --------------------------------------
class FakePerception : public BT::SyncActionNode {
public:
  // v3에서는 NodeConfiguration 타입을 사용해야 합니다 :contentReference[oaicite:0]{index=0}
  FakePerception(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  // 포트가 있으면 providedPorts를 정의해야 합니다 :contentReference[oaicite:1]{index=1}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    std::cout << "[Perception] 딸기 검출 성공!\n";
    return BT::NodeStatus::SUCCESS;
  }
};

// --------------------------------------
// 2) FakeApproach
// --------------------------------------
class FakeApproach : public BT::SyncActionNode {
public:
  FakeApproach(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    std::cout << "[Approach] 거친 접근 완료.\n";
    return BT::NodeStatus::SUCCESS;
  }
};

// --------------------------------------
// 3) FakeServoCut
// --------------------------------------
class FakeServoCut : public BT::SyncActionNode {
public:
  FakeServoCut(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    std::cout << "[Servo+Cut] IBVS servo → 컷 완료!\n";
    return BT::NodeStatus::SUCCESS;
  }
};
