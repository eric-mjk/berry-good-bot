#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <optional>
#include <string>
#include <algorithm>   // std::transform
#include <cctype>      // std::tolower
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// berry_interface actions
#include "berry_interface/action/gripper_control.hpp"
#include "berry_interface/action/visual_servoing.hpp"

namespace strawberry_bt_nodes
{

// run_bt_node.cpp 에서 세팅되는 공용 노드
extern rclcpp::Node::SharedPtr g_ros_node;

// ===============================
// GripperControlBT
//   - mode: "open" | "close"
//   - timeout: 결과 대기 최대 시간(초)
// ===============================
class GripperControlBT : public BT::SyncActionNode
{
public:
  using GripperControl = berry_interface::action::GripperControl;
  using GH = rclcpp_action::ClientGoalHandle<GripperControl>;

  GripperControlBT(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("mode", "open"),
      BT::InputPort<double>("timeout", "10.0")
    };
  }

  BT::NodeStatus tick() override
  {
    if (!g_ros_node) {
      std::cerr << "[GripperControlBT] ROS node not initialized\n";
      return BT::NodeStatus::FAILURE;
    }
    std::string mode = "open";
    (void)getInput("mode", mode);
    for (auto& c : mode) c = static_cast<char>(::tolower(c));

    const bool want_open = (mode == "open");
    double timeout_s = 10.0;
    (void)getInput("timeout", timeout_s);

    if (!client_) {
      client_ = rclcpp_action::create_client<GripperControl>(g_ros_node, "gripper_command");
    }

    using namespace std::chrono_literals;
    // 서버 대기(최대 12s, 300ms 스텝)
    auto waited = 0ms;
    const auto step  = 300ms;
    const auto limit = 12s;
    bool ready = false;
    while (rclcpp::ok() && waited < limit) {
      if (client_->wait_for_action_server(step)) { ready = true; break; }
      waited += step;
    }
    if (!ready) {
      std::cerr << "[GripperControlBT] gripper_command action server not available\n";
      return BT::NodeStatus::FAILURE;
    }

    // goal 전송
    GripperControl::Goal goal;
    goal.open = want_open;

    auto gh_fut = client_->async_send_goal(goal);
    if (gh_fut.wait_for(3s) != std::future_status::ready) {
      std::cerr << "[GripperControlBT] timeout while sending goal\n";
      return BT::NodeStatus::FAILURE;
    }
    auto gh = gh_fut.get();
    if (!gh) {
      std::cerr << "[GripperControlBT] goal rejected\n";
      return BT::NodeStatus::FAILURE;
    }

    // 결과 대기
    auto res_fut = client_->async_get_result(gh);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (res_fut.wait_for(200ms) == std::future_status::ready) break;
    }
    if (res_fut.wait_for(0s) != std::future_status::ready) {
      // 가능하면 cancel 시도
      (void)client_->async_cancel_goal(gh);
      std::cerr << "[GripperControlBT] result wait TIMEOUT (" << timeout_s << "s)\n";
      return BT::NodeStatus::FAILURE;
    }

    // 결과 검사 (가능하면 success 필드 확인)
    try {
      auto wrapped = res_fut.get();
    //   // 인터페이스에 success 필드가 있다고 가정
    //   if (wrapped.result && wrapped.result->success == false) {
    //     std::cerr << "[GripperControlBT] action returned success=false. message='"
    //               << wrapped.result->message << "'\n";
    //     return BT::NodeStatus::FAILURE;
    //   }

      // GripperControl.action의 Result는 'bool done' 입니다.
      if (!wrapped.result) {
        std::cerr << "[GripperControlBT] result is null\n";
        return BT::NodeStatus::FAILURE;
      }
      if (!wrapped.result->done) {
        std::cerr << "[GripperControlBT] action returned done=false\n";
        return BT::NodeStatus::FAILURE;
      }
    } catch (const std::exception& e) {
      std::cerr << "[GripperControlBT] get_result threw: " << e.what() << "\n";
      return BT::NodeStatus::FAILURE;
    }

    std::cout << "[GripperControlBT] " << (want_open ? "OPEN" : "CLOSE") << " done\n";
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp_action::Client<GripperControl>::SharedPtr client_;
};

// =====================================
// VisualServoingBT
//   - timeout: VisualServoing 서버에 넘길 타임아웃(초)
//   - 결과 success=false면 FAILURE로 처리
// =====================================
class VisualServoingBT : public BT::SyncActionNode
{
public:
  using VisualServoing = berry_interface::action::VisualServoing;
  using GH = rclcpp_action::ClientGoalHandle<VisualServoing>;

  VisualServoingBT(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout", "15.0"),
      // 액션 결과를 블랙보드로 내보내기
      BT::OutputPort<std::string>("fruit_class"),
      BT::OutputPort<std::string>("basket_pose")
    };
  }

  BT::NodeStatus tick() override
  {
    if (!g_ros_node) {
      std::cerr << "[VisualServoingBT] ROS node not initialized\n";
      return BT::NodeStatus::FAILURE;
    }
    double timeout_s = 15.0;
    (void)getInput("timeout", timeout_s);

    if (!client_) {
      client_ = rclcpp_action::create_client<VisualServoing>(g_ros_node, "visual_servoing");
    }

    using namespace std::chrono_literals;
    // 서버 대기(최대 12s, 300ms 스텝)
    auto waited = 0ms;
    const auto step  = 300ms;
    const auto limit = 12s;
    bool ready = false;
    while (rclcpp::ok() && waited < limit) {
      if (client_->wait_for_action_server(step)) { ready = true; break; }
      waited += step;
    }
    if (!ready) {
      std::cerr << "[VisualServoingBT] server not available\n";
      return BT::NodeStatus::FAILURE;
    }

    // goal 전송
    VisualServoing::Goal goal;
    goal.timeout_sec = timeout_s;

    // Humble API: 두 번째 인자는 SendGoalOptions여야 합니다.
    rclcpp_action::Client<VisualServoing>::SendGoalOptions send_opts;
    // (선택) 피드백 콜백
    send_opts.feedback_callback =
      [](GH::SharedPtr, const std::shared_ptr<const VisualServoing::Feedback> fb)
      {
        if (!fb) return;
        // std::cout << "[IBVS fb] t=" << fb->time_elapsed
        //           << "s  state=" << fb->state << "\n";
      };

    auto send_fut = client_->async_send_goal(goal, send_opts);
 
    if (send_fut.wait_for(3s) != std::future_status::ready) {
      std::cerr << "[VisualServoingBT] timeout while sending goal\n";
      return BT::NodeStatus::FAILURE;
    }
    // auto gh = send_fut.get();
    // if (!gh || !gh->is_active()) {
    auto gh = send_fut.get();
    // Humble: goal이 수락되면 유효한 handle, 거절되면 nullptr
    if (!gh) {
      std::cerr << "[VisualServoingBT] goal rejected or not active\n";
      return BT::NodeStatus::FAILURE;
    }

    // 결과 대기
    // auto res_fut = gh->get_result_async();
    // 결과 대기: Client::async_get_result() 사용
    auto res_fut = client_->async_get_result(gh);
    while (rclcpp::ok() && res_fut.wait_for(200ms) != std::future_status::ready) {}

    if (res_fut.wait_for(0s) != std::future_status::ready) {
    //   // 방어적으로 cancel
    //   (void)gh->cancel_goal_async();
      // 방어적으로 cancel: Client::async_cancel_goal() 사용
      (void)client_->async_cancel_goal(gh);
      std::cerr << "[VisualServoingBT] result wait aborted (node stopping?)\n";
      return BT::NodeStatus::FAILURE;
    }

    // 결과 검사
    try {
      auto wrapped = res_fut.get();
      if (!wrapped.result) {
        std::cerr << "[VisualServoingBT] result is null\n";
        return BT::NodeStatus::FAILURE;
      }
      // std::cout << "[VisualServoingBT] done: success=" << (wrapped.result->success ? "true":"false")
      //           << "  msg='" << wrapped.result->message << "'\n";
      // return wrapped.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      const bool ok = wrapped.result->success;
      const auto msg = wrapped.result->message;
      std::string fruit = wrapped.result->fruit_class;     // 새 필드
      const double conf_sum = wrapped.result->fruit_class_conf_sum; // 새 필드(로깅용)

      // 소문자 정규화
      std::string fruit_lc = fruit;
      std::transform(fruit_lc.begin(), fruit_lc.end(), fruit_lc.begin(),
                     [](unsigned char c){ return static_cast<char>(std::tolower(c)); });

      // 기본 매핑: strawberry→basket_s, apple→basket_a, orange→basket_o (그 외: basket)
      std::string basket_pose = "basket";
      if (fruit_lc == "strawberry" || fruit_lc == "berry" || fruit_lc == "strawberries" || fruit_lc == "0") {
        basket_pose = "basket_s";
      } else if (fruit_lc == "apple" || fruit_lc == "1") {
        basket_pose = "basket_a";
      } else if (fruit_lc == "orange" || fruit_lc == "2") {
        basket_pose = "basket_o";
      }

      // 블랙보드로 결과 전달(없더라도 빈 문자열/기본값 세팅)
      setOutput("fruit_class", fruit);
      setOutput("basket_pose", basket_pose);

      std::cout << "[VisualServoingBT] done: success=" << (ok ? "true":"false")
                << "  msg='" << msg << "'  fruit='" << fruit
                << "'  conf_sum=" << conf_sum
                << "  basket_pose='" << basket_pose << "'\n";

      return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    } catch (const std::exception& e) {
      std::cerr << "[VisualServoingBT] get_result threw: " << e.what() << "\n";
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp_action::Client<VisualServoing>::SharedPtr client_;
};

} // namespace strawberry_bt_nodes
