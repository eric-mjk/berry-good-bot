// /include/strawberry_task_planner/rough_approach_bt_nodes.hpp
#pragma once

#include <chrono>
#include <functional>
#include <cmath>
#include <future>
#include <optional>
#include <unordered_map>
#include <algorithm>
#include <mutex>
#include <iostream>
#include <limits>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "berry_interface/action/detect_strawberry.hpp"
#include "berry_interface/action/servo_twist.hpp"

namespace strawberry_bt_nodes {

extern rclcpp::Node::SharedPtr g_ros_node;

// ============================================================
// DetectApproachPoseBT
//  - 입력: offset_z (기본 0.05)
//  - 출력: 접근 포즈(base_link) ax,ay,az,qx,qy,qz,qw
// ============================================================
class DetectApproachPoseBT : public BT::SyncActionNode
{
public:
  using DetectStrawberry = berry_interface::action::DetectStrawberry;
  using GH = rclcpp_action::ClientGoalHandle<DetectStrawberry>;

  DetectApproachPoseBT(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("offset_z",     "camera Z-axis retreat [m]"),
      BT::InputPort<int>("stage",           "1 for first, 2 for second rough approach"),
      BT::InputPort<double>("prev_x",       "prev target x (base_link) for stage 2"),
      BT::InputPort<double>("prev_y",       "prev target y (base_link) for stage 2"),
      BT::InputPort<double>("prev_z",       "prev target z (base_link) for stage 2"),
      BT::InputPort<double>("prev_gate_m",  "gating radius (m) for stage 2"),

      BT::OutputPort<double>("ax"),
      BT::OutputPort<double>("ay"),
      BT::OutputPort<double>("az"),
      BT::OutputPort<double>("qx"),
      BT::OutputPort<double>("qy"),
      BT::OutputPort<double>("qz"),
      BT::OutputPort<double>("qw"),

      // 과실 중심(base_link)을 블랙보드로도 저장하여 다음 단계 앵커로 사용
      BT::OutputPort<double>("tx"),
      BT::OutputPort<double>("ty"),
      BT::OutputPort<double>("tz")
    };
  }

  BT::NodeStatus tick() override
  {
    if (!g_ros_node) {
      std::cerr << "[DetectApproachPose] ROS node not initialized\n";
      return BT::NodeStatus::FAILURE;
    }
    double offset = 0.05;
    int    stage  = 1;
    double prev_x=std::numeric_limits<double>::quiet_NaN();
    double prev_y=std::numeric_limits<double>::quiet_NaN();
    double prev_z=std::numeric_limits<double>::quiet_NaN();
    double prev_gate = 0.12;

    (void)getInput<double>("offset_z", offset);
    (void)getInput<int>("stage", stage);
    (void)getInput<double>("prev_x", prev_x);
    (void)getInput<double>("prev_y", prev_y);
    (void)getInput<double>("prev_z", prev_z);
    (void)getInput<double>("prev_gate_m", prev_gate);

    if (!client_) {
      client_ = rclcpp_action::create_client<DetectStrawberry>(g_ros_node, "detect_strawberry");
    }
    // using namespace std::chrono_literals;
    // if (!client_->wait_for_action_server(3s)) {
    //   std::cerr << "[DetectApproachPose] action server not available\n";
    //   return BT::NodeStatus::FAILURE;
    // }
    using namespace std::chrono_literals;
    // Robust wait with retries (discovery can take a few seconds)
    const auto step   = 300ms;
    const auto total  = 12s;
    auto waited = 0ms;
    bool ready = false;
    while (rclcpp::ok() && waited < total) {
      if (client_->wait_for_action_server(step)) { ready = true; break; }
      waited += step;
    }
    if (!ready) {
      std::cerr << "[DetectApproachPose] action server not available after "
                << std::chrono::duration_cast<std::chrono::seconds>(waited).count()
                << "s\n";
      return BT::NodeStatus::FAILURE;
    }

    DetectStrawberry::Goal goal;
    goal.offset_z = offset;
    goal.approach_stage = static_cast<uint8_t>(stage);
    goal.prev_target_base.x = std::isfinite(prev_x) ? prev_x : 0.0;
    goal.prev_target_base.y = std::isfinite(prev_y) ? prev_y : 0.0;
    goal.prev_target_base.z = std::isfinite(prev_z) ? prev_z : 0.0;
    goal.prev_gate_m = prev_gate;

    auto gh_fut = client_->async_send_goal(goal);
    if (gh_fut.wait_for(5s) != std::future_status::ready) {
      std::cerr << "[DetectApproachPose] timeout while sending goal\n";
      return BT::NodeStatus::FAILURE;
    }
    auto gh = gh_fut.get();
    if (!gh) {
      std::cerr << "[DetectApproachPose] goal was rejected\n";
      return BT::NodeStatus::FAILURE;
    }
    auto res_fut = client_->async_get_result(gh);
    if (res_fut.wait_for(30s) != std::future_status::ready) {
      std::cerr << "[DetectApproachPose] timeout waiting result\n";
      return BT::NodeStatus::FAILURE;
    }
    auto wrapped = res_fut.get();

    // 1) 액션 결과 코드 확인 (SUCCEEDED 외에는 실패 취급)
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      std::cerr << "[DetectApproachPose] action finished with code="
                << static_cast<int>(wrapped.code)
                << " (expect SUCCEEDED). Failing BT node.\n";
      return BT::NodeStatus::FAILURE;
    }

    const auto& ps = wrapped.result->approach_pose_base;
    const auto& tc = wrapped.result->strawberry_center_base;

    // 2) 결과 포즈 유효성 검사 (NaN/inf 및 원점(0,0,0) 보호)
    const double ax = ps.pose.position.x;
    const double ay = ps.pose.position.y;
    const double az = ps.pose.position.z;
    const bool finite_pos = std::isfinite(ax) && std::isfinite(ay) && std::isfinite(az);
    const double norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (!finite_pos || norm < 1e-6) {
      std::cerr << "[DetectApproachPose] invalid approach pose from action "
                << "(pos=" << ax << "," << ay << "," << az << "). Failing.\n";
      return BT::NodeStatus::FAILURE;
    }
    // 블랙보드로 출력
    setOutput("ax", ax);
    setOutput("ay", ay);
    setOutput("az", az);
    setOutput("qx", ps.pose.orientation.x);
    setOutput("qy", ps.pose.orientation.y);
    setOutput("qz", ps.pose.orientation.z);
    setOutput("qw", ps.pose.orientation.w);

    // 과실 중심(다음 단계 앵커)
    setOutput("tx", tc.x);
    setOutput("ty", tc.y);
    setOutput("tz", tc.z);


    std::cout << "[DetectApproachPose] approach(base_link): "
              << ax << ", " << ay << ", " << az << "\n";
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp_action::Client<DetectStrawberry>::SharedPtr client_;
};

// ============================================================
// ServoToTargetPoseBT
//  - 입력: 목표 포즈(base_link) ax..qw, 여러 파라미터
//  - 동작: ServoTwist 액션 시작 → /eef_twist_cmd로 Twist 발행
//          TF(base_link→link5)로 현재 포즈 읽고 수렴 판정
// ============================================================
class ServoToTargetPoseBT : public BT::SyncActionNode
{
public:
  using ServoTwist = berry_interface::action::ServoTwist;
  using GHServo    = rclcpp_action::ClientGoalHandle<ServoTwist>;

  ServoToTargetPoseBT(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg)
  {
    if (!g_ros_node) throw std::runtime_error("ServoToTargetPoseBT: ROS node not initialized");
    pub_twist_ = g_ros_node->create_publisher<geometry_msgs::msg::Twist>("/eef_twist_cmd", 10);
    tfbuf_ = std::make_unique<tf2_ros::Buffer>(g_ros_node->get_clock());
    tfl_   = std::make_unique<tf2_ros::TransformListener>(*tfbuf_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      // target pose
      BT::InputPort<double>("ax"), BT::InputPort<double>("ay"), BT::InputPort<double>("az"),
      BT::InputPort<double>("qx"), BT::InputPort<double>("qy"), BT::InputPort<double>("qz"), BT::InputPort<double>("qw"),
      // frames
      BT::InputPort<std::string>("base_frame", "base_link"),
      BT::InputPort<std::string>("tip_frame",  "link5"),
      // gains & limits
      BT::InputPort<double>("kp_pos",  "0.8"),
      BT::InputPort<double>("kp_ori",  "1.2"),
      BT::InputPort<double>("vmax_lin","0.15"),
      BT::InputPort<double>("vmin_lin","0.03"),
      BT::InputPort<double>("vmax_ang","1.0"),
      BT::InputPort<double>("vmin_ang","0.20"),
      // tip offset: use a virtual frame translated along tip Z (gripper end-point)
      BT::InputPort<double>("tip_offset_z", "0.13"),
      // stopping
      BT::InputPort<double>("pos_tol",    "0.01"),
      BT::InputPort<double>("ori_tol",    "0.05"),
      BT::InputPort<double>("stable_time","0.5"),
      BT::InputPort<double>("timeout",    "25.0")
    };
  }

  BT::NodeStatus tick() override
  {
    // --- 입력 읽기 ---
    double ax, ay, az, qx, qy, qz, qw;
    if (!getInput("ax", ax) || !getInput("ay", ay) || !getInput("az", az) ||
        !getInput("qx", qx) || !getInput("qy", qy) || !getInput("qz", qz) || !getInput("qw", qw)) {
      std::cerr << "[ServoToTargetPose] missing target pose inputs\n";
      return BT::NodeStatus::FAILURE;
    }
    std::string base = "base_link", tip = "link5";
    (void)getInput("base_frame", base);
    (void)getInput("tip_frame",  tip);
    double kp_pos=0.8, kp_ori=1.2, vmax_lin=0.15, vmax_ang=1.0;
    (void)getInput("kp_pos", kp_pos);
    (void)getInput("kp_ori", kp_ori);
    (void)getInput("vmax_lin", vmax_lin);
    double vmin_lin=0.03; (void)getInput("vmin_lin", vmin_lin);
    (void)getInput("vmax_ang", vmax_ang);
    double vmin_ang=0.20; (void)getInput("vmin_ang", vmin_ang);
    // how far the virtual "gripper end" is from link5 origin along +Z_tip
    double tip_off_z=0.13; (void)getInput("tip_offset_z", tip_off_z);
    double pos_tol=0.01, ori_tol=0.05, stable_s=0.5, timeout_s=25.0;
    (void)getInput("pos_tol", pos_tol);
    (void)getInput("ori_tol", ori_tol);
    (void)getInput("stable_time", stable_s);
    (void)getInput("timeout", timeout_s);

    // --- ServoTwist 액션 시작 ---
    if (!servo_client_) {
      servo_client_ = rclcpp_action::create_client<ServoTwist>(g_ros_node, "servo_twist");
    }
    // using namespace std::chrono_literals;
    // if (!servo_client_->wait_for_action_server(3s)) {
    //   std::cerr << "[ServoToTargetPose] servo action server not available\n";
    //   return BT::NodeStatus::FAILURE;
    // }

    using namespace std::chrono_literals;
    // Same robust wait pattern
    const auto step   = 300ms;
    const auto total  = 12s;
    auto waited = 0ms;
    bool ready = false;
    while (rclcpp::ok() && waited < total) {
      if (servo_client_->wait_for_action_server(step)) { ready = true; break; }
      waited += step;
    }
    if (!ready) {
      std::cerr << "[ServoToTargetPose] servo action server not available after "
                << std::chrono::duration_cast<std::chrono::seconds>(waited).count()
                << "s\n";
      return BT::NodeStatus::FAILURE;
    }
    auto gh_fut = servo_client_->async_send_goal(ServoTwist::Goal());
    if (gh_fut.wait_for(3s) != std::future_status::ready) {
      std::cerr << "[ServoToTargetPose] timeout while sending servo goal\n";
      return BT::NodeStatus::FAILURE;
    }
    auto gh = gh_fut.get();
    if (!gh) {
      std::cerr << "[ServoToTargetPose] servo goal rejected\n";
      return BT::NodeStatus::FAILURE;
    }

    // ── 정리 헬퍼: 제로트위스트 N회 발행 후 goal 취소(+결과 드레인) ───────────────
    auto publish_zero_twist = [&](int n, double dt_sec){
      geometry_msgs::msg::Twist z;
      // rclcpp::sleep_for는 nanoseconds만 받음 → 변환 필요
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::duration<double>(dt_sec));
      for(int i=0;i<n;i++){
        pub_twist_->publish(z);
        rclcpp::sleep_for(ns);
      }
    };

    auto stop_servo = [&](){
      // 최신 twist를 0으로 덮어쓰기 (servo_active 가 남아있어도 더 이상 움직이지 않게)
      publish_zero_twist(5, 0.02); // 5회 × 20ms

      // 1) cancel 요청 전송
      auto cancel_fut = servo_client_->async_cancel_goal(gh);
      if (cancel_fut.wait_for(2s) == std::future_status::ready) {
        // 반환값을 실제로 get() 해서 서비스 응답을 드레인
        (void)cancel_fut.get();
      } else {
        std::cerr << "[ServoToTargetPose] warn: cancel_goal not confirmed within 2s\n";
      }

      // 2) 서버 코루틴이 완전히 끝날 때까지 잠깐 대기 (clean shutdown)
      auto res_fut = servo_client_->async_get_result(gh);
      (void)res_fut.wait_for(2s);
    };

    // 어떤 경로로 함수를 빠져나가든 stop_servo()가 보장되도록 가드 설치
    struct CancelGuard {
      std::function<void()> f; bool active{true};
      ~CancelGuard(){ if(active && f) f(); }
    } guard{stop_servo, true};

    auto clamp_vec_minmax = [](tf2::Vector3& v, double vmin, double vmax){
      const double n = v.length();
      if (n < 1e-12) return;                 // 0 벡터는 그대로
      double m = std::clamp(n, vmin, vmax);  // 크기만 vmin~vmax로
      if (std::isfinite(m) && n > 0.0) {
        v *= (m / n);
      }
    };


    // 목표 쿼터니언
    tf2::Quaternion q_target(qx, qy, qz, qw);
    q_target.normalize();

    auto t0 = std::chrono::steady_clock::now();
    std::optional<std::chrono::steady_clock::time_point> hold_start;
    double last_pos_err = std::numeric_limits<double>::quiet_NaN();
    double last_ori_err = std::numeric_limits<double>::quiet_NaN();

    // --- 제어 루프 ---
    rclcpp::Rate rate(20.0); // 20 Hz
    while (rclcpp::ok()) {
      // timeout?
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (elapsed > timeout_s) {
        std::cerr << "[ServoToTargetPose] TIMEOUT after " << elapsed
                  << "s (timeout=" << timeout_s << "s), "
                  << "pos_err=" << (std::isfinite(last_pos_err)? last_pos_err : -1)
                  << ", ori_err=" << (std::isfinite(last_ori_err)? last_ori_err : -1)
                  << "\n";

                  
        // 안전 정지는 guard 소멸자에서 수행됨 (제로 트위스트 → cancel → result 드레인)
        return BT::NodeStatus::FAILURE;
      }

      // 현재 링크 포즈 (base→tip)
      geometry_msgs::msg::TransformStamped tf_bt;
      try {
        tf_bt = tfbuf_->lookupTransform(base, tip, tf2::TimePointZero);
      } catch (const std::exception& e) {
        std::cerr << "[ServoToTargetPose] TF lookup failed: " << e.what() << "\n";
        rate.sleep();
        continue;
      }
      const auto& t = tf_bt.transform.translation;
      const auto& r = tf_bt.transform.rotation;
      tf2::Quaternion q_cur(r.x, r.y, r.z, r.w);
      q_cur.normalize();

      // // 위치/자세 오차
      // const tf2::Vector3 p_cur(t.x, t.y, t.z);
      // 위치/자세 오차
      // Use a virtual "tip" translated along the current tip Z by tip_off_z (e.g., 0.15 m)
      const tf2::Vector3 p_tip(t.x, t.y, t.z);
      // z_tip = R_tip * [0,0,1]  (use quatRotate for robustness)
      const tf2::Vector3 z_tip = tf2::quatRotate(q_cur, tf2::Vector3(0.0, 0.0, 1.0));
      const tf2::Vector3 p_eff = p_tip + z_tip * tip_off_z;
      const tf2::Vector3 p_tgt(ax, ay, az);
      tf2::Vector3 p_err = p_tgt - p_eff;
      last_pos_err = p_err.length();

      // quat error: q_err = q_tgt * inv(q_cur) (axis-angle)
      tf2::Quaternion q_err = q_target * q_cur.inverse();
      q_err.normalize();
     double sin_half = std::sqrt(q_err.x()*q_err.x() + q_err.y()*q_err.y() + q_err.z()*q_err.z());
      double angle = 2.0 * std::atan2(sin_half, std::max(1e-9, (double)q_err.w()));
      if (angle > M_PI) angle -= 2*M_PI;
      last_ori_err = std::fabs(angle);
      tf2::Vector3 axis(0,0,0);
     if (sin_half > 1e-9) {
        axis = tf2::Vector3(q_err.x(), q_err.y(), q_err.z()) / sin_half;
      }
      tf2::Vector3 w_err = axis * angle;  // base frame

    //   // P 제어 + 속도 제한
    //   tf2::Vector3 v = p_err * kp_pos;
    //   tf2::Vector3 w = w_err * kp_ori;
    //   limit_vec(v, vmax_lin);
    //   limit_vec(w, vmax_ang);

    //   geometry_msgs::msg::Twist tw;
    //   tw.linear.x = v.x(); tw.linear.y = v.y(); tw.linear.z = v.z();
    //   tw.angular.x = w.x(); tw.angular.y = w.y(); tw.angular.z = w.z();
    //   pub_twist_->publish(tw);

    //   // 수렴 판정
    //   const bool pos_ok = (last_pos_err <= pos_tol);
    //   const bool ori_ok = (last_ori_err <= ori_tol);

      // 수렴 판정 (먼저)
      const bool pos_ok = (last_pos_err <= pos_tol);
      const bool ori_ok = (last_ori_err <= ori_tol);
      geometry_msgs::msg::Twist tw;

      if (pos_ok && ori_ok) {
        // 오차 허용내 → 제로 트위스트 유지 (덜덜 방지)
        tw.linear.x = tw.linear.y = tw.linear.z = 0.0;
        tw.angular.x = tw.angular.y = tw.angular.z = 0.0;
        pub_twist_->publish(tw);
      } else {
        // P 제어 + (최저~최대) 속도 제한
        tf2::Vector3 v = p_err * kp_pos;
        tf2::Vector3 w = w_err * kp_ori;
        clamp_vec_minmax(v, vmin_lin, vmax_lin);
        clamp_vec_minmax(w, vmin_ang, vmax_ang);
        tw.linear.x = v.x(); tw.linear.y = v.y(); tw.linear.z = v.z();
        tw.angular.x = w.x(); tw.angular.y = w.y(); tw.angular.z = w.z();
        pub_twist_->publish(tw);
      }

      if (pos_ok && ori_ok) {
        if (!hold_start.has_value()) {
          hold_start = std::chrono::steady_clock::now();
        } else {
          const double held = std::chrono::duration<double>(std::chrono::steady_clock::now() - *hold_start).count();
          if (held >= stable_s) {
            std::cout << "[ServoToTargetPose] CONVERGED  pos_err=" << last_pos_err
                      << " (tol=" << pos_tol << "), ori_err=" << last_ori_err
                      << " (tol=" << ori_tol << "), waited=" << elapsed
                      << "s (timeout=" << timeout_s << "s)\n";

            // 안전 정지는 guard 소멸자에서 보장됨.
            // 여기서는 SUCCESS 만 반환.
                      
            return BT::NodeStatus::SUCCESS;
          }
        }
      } else {
        hold_start.reset();
      }

      rate.sleep();
    }

    // 노드 종료(rclcpp::ok() false 등) → 안전 정지
    // guard 가 stop_servo()를 호출할 것이므로 추가 조치 불필요.
    return BT::NodeStatus::FAILURE;
  }

private:
  static void limit_vec(tf2::Vector3& v, double vmax)
  {
    const double n = v.length();
    if (n > vmax && n > 1e-12) v *= (vmax / n);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::unique_ptr<tf2_ros::TransformListener> tfl_;
  rclcpp_action::Client<ServoTwist>::SharedPtr servo_client_;
};

} // namespace strawberry_bt_nodes
