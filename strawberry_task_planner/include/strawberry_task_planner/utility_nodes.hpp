// include/strawberry_task_planner/utility_nodes.hpp
#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <cctype>      // std::isspace
#include <iterator>    // std::distance
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

namespace strawberry_bt_nodes {

// 안전한 문자열 트림 (앞/뒤 공백 제거)
inline std::string trim(const std::string& s){
  if (s.empty()) return {};
  auto b = s.begin();
  while (b != s.end() && std::isspace(static_cast<unsigned char>(*b))) ++b;
  if (b == s.end()) return {};  // 모두 공백
  auto e = s.end();
  do { --e; } while (e >= b && std::isspace(static_cast<unsigned char>(*e)));
  return (b <= e) ? std::string(b, e + 1) : std::string();
}

inline std::vector<std::string> split_csv(const std::string& csv){
  std::vector<std::string> out;
  std::stringstream ss(csv);
  std::string item;
  while(std::getline(ss, item, ',')){
    auto t = trim(item);
    if(!t.empty()) out.push_back(t);
  }
  return out;
}

// PoseListInit
//  - pose_list="ready,ready1,ready2"
//  - idx_key: 블랙보드 인덱스 키(기본 "ready_idx")
//  - 출력: current("{ready_pose}"), first("{first_ready}")
class PoseListInit : public BT::SyncActionNode
{
public:
  PoseListInit(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("pose_list"),
      BT::InputPort<std::string>("idx_key", "ready_idx"),
      BT::OutputPort<std::string>("current"),
      BT::OutputPort<std::string>("first")
    };
  }

  BT::NodeStatus tick() override {
    std::string csv; if(!getInput("pose_list", csv) || csv.empty()){
      std::cerr << "[PoseListInit] pose_list is empty\n";
      return BT::NodeStatus::FAILURE;
    }
    std::string idx_key = "ready_idx"; (void)getInput("idx_key", idx_key);
    auto list = split_csv(csv);
    if(list.empty()){
      std::cerr << "[PoseListInit] parsed empty list\n";
      return BT::NodeStatus::FAILURE;
    }
    // index 0으로 초기화
    config().blackboard->set<int>(idx_key, 0);
    setOutput("current", list.front());
    setOutput("first",   list.front());
    return BT::NodeStatus::SUCCESS;
  }
};

// PoseListNext
//  - pose_list="ready,ready1,ready2"
//  - idx_key(기본 "ready_idx")를 블랙보드에서 읽어 +1
//  - 성공 시 current("{ready_pose}") 갱신, 더 이상 없으면 FAILURE
class PoseListNext : public BT::SyncActionNode
{
public:
  PoseListNext(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("pose_list"),
      BT::InputPort<std::string>("idx_key", "ready_idx"),
      BT::OutputPort<std::string>("current")
    };
  }

  BT::NodeStatus tick() override {
    std::string csv; if(!getInput("pose_list", csv) || csv.empty()){
      std::cerr << "[PoseListNext] pose_list is empty\n";
      return BT::NodeStatus::FAILURE;
    }
    auto list = split_csv(csv);
    if(list.empty()){
      std::cerr << "[PoseListNext] parsed empty list\n";
      return BT::NodeStatus::FAILURE;
    }
    std::string idx_key = "ready_idx"; (void)getInput("idx_key", idx_key);
    int idx = 0;
    if(!config().blackboard->get<int>(idx_key, idx)){
      idx = 0;
    }
    idx += 1; // 다음 포즈
    if(idx >= static_cast<int>(list.size())){
      std::cerr << "[PoseListNext] no more poses (size=" << list.size() << ")\n";
      return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set<int>(idx_key, idx);
    setOutput("current", list[idx]);
    return BT::NodeStatus::SUCCESS;
  }
};

// ------------------------------------------------------------
// FruitToBasketPose
//  - 입력: fruit("{fruit_class}")
//  - 출력: basket("{basket_pose}")  // 예: strawberry->basket_s, apple->basket_a, orange->basket_o
//  - 매핑이 필요 없으면 VisualServoingBT에서 이미 세팅되므로 생략 가능.
// ------------------------------------------------------------
class FruitToBasketPose : public BT::SyncActionNode
{
public:
  FruitToBasketPose(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("fruit"),
      BT::OutputPort<std::string>("basket")
    };
  }

  BT::NodeStatus tick() override {
    std::string fruit; (void)getInput("fruit", fruit);
    std::string lc = fruit;
    std::transform(lc.begin(), lc.end(), lc.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    std::string pose = "basket";
    if (lc == "strawberry" || lc == "berry" || lc == "strawberries" || lc == "0")      pose = "basket_s";
    else if (lc == "apple" || lc == "1")                                               pose = "basket_a";
    else if (lc == "orange" || lc == "2")                                              pose = "basket_o";
    setOutput("basket", pose);
    return BT::NodeStatus::SUCCESS;
  }
};

// ------------------------------------------------------------
// WaitSeconds
//  - 입력: seconds="2.0" (기본 1.0)
//  - 논블로킹 대기: 최초 tick에서 시작시간 기록 → 경과 전에는 RUNNING, 지나면 SUCCESS
//  - 그리퍼 OPEN 직후 "그대로 멈춘 상태로 몇 초 대기" 용
// ------------------------------------------------------------
class WaitSeconds : public BT::StatefulActionNode
{
public:
  WaitSeconds(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts(){
    return { BT::InputPort<double>("seconds", "1.0") };
  }

  BT::NodeStatus onStart() override {
    if(!getInput("seconds", seconds_)) {
      seconds_ = 1.0;
    }
    start_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_).count();
    if (elapsed >= seconds_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
    // do nothing
  }

private:
  double seconds_{1.0};
  std::chrono::steady_clock::time_point start_;
};


} // namespace strawberry_bt_nodes
