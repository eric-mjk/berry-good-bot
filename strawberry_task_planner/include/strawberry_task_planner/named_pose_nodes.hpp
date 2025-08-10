// include/strawberry_task_planner/named_pose_nodes.hpp
#pragma once

#include <string>
#include <iostream>
#include <chrono>
#include <future>
#include <mutex>
#include <unordered_map>
#include <optional>
#include <limits>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "berry_interface/action/move_to_named_pose.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

namespace strawberry_bt_nodes {

// run_bt_node.cpp 에서 설정하는 전역 node 공유
extern rclcpp::Node::SharedPtr g_ros_node;

class MoveToNamedPoseBT : public BT::SyncActionNode
{
public:
  using MoveToNamedPose = berry_interface::action::MoveToNamedPose;
  using GoalHandleMP    = rclcpp_action::ClientGoalHandle<MoveToNamedPose>;

  MoveToNamedPoseBT(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
  {
    // 액션 클라이언트 생성은 tick에서 lazy-init 해도 되지만 여기서 바로 만들어도 됨
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("pose", "named pose name") };
  }

  BT::NodeStatus tick() override
  {
    if (!g_ros_node) {
      std::cerr << "[MoveToNamedPose] ROS node not initialized\n";
      return BT::NodeStatus::FAILURE;
    }

    // 입력 포트에서 포즈 이름 가져오기
    std::string pose_name;
    if (!getInput<std::string>("pose", pose_name)) {
      std::cerr << "[MoveToNamedPose] missing required input port: 'pose'\n";
      return BT::NodeStatus::FAILURE;
    }

    // 액션 클라이언트 lazy init
    if (!client_) {
      client_ = rclcpp_action::create_client<MoveToNamedPose>(g_ros_node, "move_to_named_pose");
    }

    // using namespace std::chrono_literals;
    // if (!client_->wait_for_action_server(2s)) {
    //   std::cerr << "[MoveToNamedPose] action server not available\n";
    //   return BT::NodeStatus::FAILURE;
    // }

    using namespace std::chrono_literals;
    // Robust wait: retry in small steps up to ~12s total
    const auto step   = 300ms;
    const auto total  = 12s;
    auto waited       = 0ms;
    bool ready        = false;
    while (rclcpp::ok() && waited < total) {
      if (client_->wait_for_action_server(step)) { ready = true; break; }
      waited += step;
    }
    if (!ready) {
      std::cerr << "[MoveToNamedPose] action server not available after "
                << std::chrono::duration_cast<std::chrono::seconds>(waited).count()
                << "s\n";
      return BT::NodeStatus::FAILURE;
    }
    // Goal 전송
    MoveToNamedPose::Goal goal;
    goal.pose_name = pose_name;

    std::cout << "[MoveToNamedPose] Sending named pose: " << pose_name << std::endl;

    // auto goal_future = client_->async_send_goal(goal);
    // if (rclcpp::spin_until_future_complete(g_ros_node, goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //   std::cerr << "[MoveToNamedPose] failed to send goal\n";
    //   return BT::NodeStatus::FAILURE;
    // }
    auto goal_future = client_->async_send_goal(goal);
    using namespace std::chrono_literals;
    if (goal_future.wait_for(5s) != std::future_status::ready) {
      std::cerr << "[MoveToNamedPose] timeout while sending goal\n";
      return BT::NodeStatus::FAILURE;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      std::cerr << "[MoveToNamedPose] goal was rejected by server\n";
      return BT::NodeStatus::FAILURE;
    }

    // 결과 대기
    // auto result_future = client_->async_get_result(goal_handle);
    // if (rclcpp::spin_until_future_complete(g_ros_node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //   std::cerr << "[MoveToNamedPose] failed to get result\n";
    //   return BT::NodeStatus::FAILURE;
    // }
    auto result_future = client_->async_get_result(goal_handle);
    if (result_future.wait_for(30s) != std::future_status::ready) {
      std::cerr << "[MoveToNamedPose] timeout while waiting for result\n";
      return BT::NodeStatus::FAILURE;
    }

    auto wrapped = result_future.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      std::cout << "[MoveToNamedPose] SUCCESS (" << pose_name << ")\n";
      return BT::NodeStatus::SUCCESS;
    } else {
      std::cerr << "[MoveToNamedPose] FAILURE (" << pose_name << "), code=" << static_cast<int>(wrapped.code) << "\n";
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp_action::Client<MoveToNamedPose>::SharedPtr client_;
};

// ================================================================
//  WaitRealRobotSync
//    /joint_states (목표; RViz/MoveIt)  vs  /current_joint_states (실로봇 피드백)
//    → |err| <= tolerance 가 stable_time 동안 유지되면 SUCCESS, timeout 시 FAILURE
// ================================================================
class WaitRealRobotSync : public BT::SyncActionNode
{
public:
  WaitRealRobotSync(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg)
  {
    if (!g_ros_node) {
      throw std::runtime_error("WaitRealRobotSync: ROS node not initialized");
    }
    using std::placeholders::_1;
    sub_joint_ = g_ros_node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&WaitRealRobotSync::joint_cb, this, _1));
    sub_curr_  = g_ros_node->create_subscription<sensor_msgs::msg::JointState>(
      "/current_joint_states", 10, std::bind(&WaitRealRobotSync::curr_cb, this, _1));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("tolerance", "per-joint abs error tolerance"),
      BT::InputPort<double>("stable_time", "seconds hold within tolerance"),
      BT::InputPort<double>("timeout", "seconds to give up")
    };
  }

  BT::NodeStatus tick() override
  {
    // defaults
    double tol = 0.01, stable_s = 0.5, timeout_s = 20.0;
    (void)getInput<double>("tolerance", tol);
    (void)getInput<double>("stable_time", stable_s);
    (void)getInput<double>("timeout", timeout_s);

    auto t0 = std::chrono::steady_clock::now();
    std::optional<std::chrono::steady_clock::time_point> hold_start;
    // 마지막으로 계산된 최대 오차(로그용)
    double last_max_err = std::numeric_limits<double>::quiet_NaN();

    // 대기 루프
    while (true) {
      // timeout?
    //   if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s) {
    //     std::cerr << "[WaitRealRobotSync] timeout\n";
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (elapsed > timeout_s) {
        std::cerr << "[WaitRealRobotSync] TIMEOUT after "
                  << elapsed << "s (timeout=" << timeout_s
                  << "s), last_err=";
        if (std::isfinite(last_max_err)) {
          std::cerr << last_max_err;
        } else {
          std::cerr << "n/a";
        }
        std::cerr << ", tol=" << tol << "\n";
        return BT::NodeStatus::FAILURE;
      }

      // 최신 샘플 확보
      std::unordered_map<std::string, double> tgt, cur;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        tgt = last_joint_;
        cur = last_curr_;
      }
      if (tgt.empty() || cur.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      // 교집합 기준으로 최대 오차 계산
      double max_err = 0.0;
      size_t n_used = 0;
     for (const auto& [name, tgt_pos] : tgt) {
        auto it = cur.find(name);
        if (it == cur.end()) continue;
        double err = std::abs(tgt_pos - it->second);
        if (err > max_err) max_err = err;
        ++n_used;
      }
      // 로그용으로 마지막 max_err 갱신
      if (n_used > 0) {
        last_max_err = max_err;
      }
      if (n_used == 0) {
        std::cerr << "[WaitRealRobotSync] no common joints between /joint_states and /current_joint_states\n";
        return BT::NodeStatus::FAILURE;
      }

      // 수렴 판정
      if (max_err <= tol) {
        if (!hold_start.has_value()) {
          hold_start = std::chrono::steady_clock::now();
        } else {
          double held = std::chrono::duration<double>(std::chrono::steady_clock::now() - *hold_start).count();
          if (held >= stable_s) {
            // std::cout << "[WaitRealRobotSync] CONVERGED  max_err=" << max_err << " (<= " << tol << ")\n";
            const double total_wait = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            std::cout << "[WaitRealRobotSync] CONVERGED  max_err=" << max_err
                      << " (<= " << tol << "), waited=" << total_wait
                      << "s (timeout=" << timeout_s << "s)\n";
            return BT::NodeStatus::SUCCESS;
          }
        }
      } else {
        hold_start.reset(); // 수렴 깨짐 → 다시 측정
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

private:
  void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_joint_.clear();
    for (size_t i=0;i<msg->name.size() && i<msg->position.size();++i) {
      last_joint_[msg->name[i]] = msg->position[i];
    }
  }
  void curr_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_curr_.clear();
    for (size_t i=0;i<msg->name.size() && i<msg->position.size();++i) {
      last_curr_[msg->name[i]] = msg->position[i];
    }
  }

  std::mutex mtx_;
  std::unordered_map<std::string, double> last_joint_;
  std::unordered_map<std::string, double> last_curr_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_curr_;
};

} // namespace strawberry_bt_nodes
