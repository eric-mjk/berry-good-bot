// include/strawberry_task_planner/rough_approach_nodes.hpp
#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <random>
#include <Eigen/Geometry> // Eigen::Isometry3d
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // geometry_msgs::msg::PoseStamped to Eigen::Isometry3d

#include <visualization_msgs/msg/marker.hpp>  // ⬅ PerceptionVisualizer가

namespace strawberry_bt_nodes {
  extern rclcpp::Node::SharedPtr g_ros_node;

// Helper function to create a Pose from x,y,z and orientation
geometry_msgs::msg::Pose createPose(double x, double y, double z, double qw = 1.0, double qx = 0.0, double qy = 0.0, double qz = 0.0) {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = qw;
    p.orientation.x = qx;
    p.orientation.y = qy;
    p.orientation.z = qz;
    return p;
}

// --------------------------------------
// 1) TestPerception Node
// --------------------------------------
class TestPerception : public BT::SyncActionNode {
public:
  TestPerception(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
        // 랜덤 시드 설정
        std::random_device rd;
        gen = std::mt19937(rd());
        // 로봇의 대략적인 작업 공간 정의 (SRDF와 로봇 구성에 따라 조절 필요)
        // 예를 들어, 로봇 베이스로부터의 거리와 높이
        x_dist = std::uniform_real_distribution<>(-0.5, 0.5); // 예시: x 범위
        y_dist = std::uniform_real_distribution<>(-0.5, 0.5); // 예시: y 범위
        z_dist = std::uniform_real_distribution<>(0.2, 0.8);  // 예시: z 범위 (바닥 위)
    }

  static BT::PortsList providedPorts() {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Generated random target pose")
    };
  }

  BT::NodeStatus tick() override {
    geometry_msgs::msg::PoseStamped target_pose_stamped;
    target_pose_stamped.header.frame_id = "base_link"; // 로봇의 베이스 링크 기준
    target_pose_stamped.header.stamp = rclcpp::Clock().now();

    // 로봇의 SRDF를 고려한 랜덤 위치 생성
    // 임의의 범위 설정. 실제 로봇과 Workspace에 맞춰 조절해야 합니다.
    // 여기서는 로봇 중심에서 특정 범위 내의 위치를 생성합니다.
    target_pose_stamped.pose.position.x = x_dist(gen);
    target_pose_stamped.pose.position.y = y_dist(gen);
    target_pose_stamped.pose.position.z = z_dist(gen);

    // 로봇 중심에서 바깥을 쳐다보는 방향 (예시: x축 방향)
    // 일반적으로 End-Effector의 Orientation은 복잡하지만, 여기서는 단순화.
    // StrawberryPerception의 경우, 딸기를 잡기 위한 특정 방향이 필요할 수 있습니다.
    // 예를 들어, x축을 따라 앞을 향하는 방향.
    target_pose_stamped.pose.orientation.x = 0.0;
    target_pose_stamped.pose.orientation.y = 0.0;
    target_pose_stamped.pose.orientation.z = 0.0;
    target_pose_stamped.pose.orientation.w = 1.0; // Identity (no rotation)

    // 만약 "로봇 중심에서 바깥을 쳐다보는 방향"을 더 정확히 구현하려면
    // 예를 들어, EEF가 항상 Z-축이 아래를 향하고 X-축이 목표를 향하도록 설정
    // Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()) * // -90 deg around Y to point X forward
    //                                         Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())); // +90 deg around Z if needed
    // target_pose_stamped.pose.orientation.x = q.x();
    // target_pose_stamped.pose.orientation.y = q.y();
    // target_pose_stamped.pose.orientation.z = q.z();
    // target_pose_stamped.pose.orientation.w = q.w();


    setOutput("target_pose", target_pose_stamped);
    RCLCPP_INFO(rclcpp::get_logger("TestPerception"),
                "Generated target pose: x=%.2f, y=%.2f, z=%.2f",
                target_pose_stamped.pose.position.x,
                target_pose_stamped.pose.position.y,
                target_pose_stamped.pose.position.z);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::mt19937 gen;
  std::uniform_real_distribution<> x_dist, y_dist, z_dist;
};

// --------------------------------------
// 2) PerceptionVisualizer Node
// --------------------------------------

// include/strawberry_task_planner/rough_approach_nodes.hpp 중 일부
class PerceptionVisualizer : public BT::SyncActionNode
{
public:
  PerceptionVisualizer(const std::string& name,
                       const BT::NodeConfiguration& cfg)
      : BT::SyncActionNode(name, cfg),
        node_(strawberry_bt_nodes::g_ros_node)
  {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", rclcpp::QoS(rclcpp::KeepLast(10))
                                   .transient_local());
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose")};
  }

  BT::NodeStatus tick() override
  {
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
    if (!pose)
    {
      RCLCPP_ERROR(node_->get_logger(), "target_pose 입력 누락: %s",
                   pose.error().c_str());
      return BT::NodeStatus::FAILURE;
    }

    visualization_msgs::msg::Marker m;
    m.header = pose->header;
    m.ns     = "perception_targets";
    m.id     = 0;
    m.type   = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose   = pose->pose;
    // m.scale  = {0.05, 0.05, 0.05};
    // m.color  = {1.0, 0.0, 0.0, 1.0};
    // Vector3 및 ColorRGBA 필드별로 할당
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;

    marker_pub_->publish(m);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};
class RoughApproach : public BT::StatefulActionNode
{
public:
  RoughApproach(const std::string& name,
                const BT::NodeConfiguration& cfg)
      : BT::StatefulActionNode(name, cfg),
        node_(strawberry_bt_nodes::g_ros_node)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, "manipulator");
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(3);

    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        node_, "base_link", "/moveit_visual_tools",
        move_group_->getRobotModel());
    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose")};
  }

  /** onStart → plan,  onRunning → execute */
  BT::NodeStatus onStart() override
  {
    if (!getInput("target_pose", target_pose_))
    {
      RCLCPP_ERROR(node_->get_logger(), "target_pose 입력 없음");
      return BT::NodeStatus::FAILURE;
    }
    move_group_->setPoseTarget(target_pose_.pose);
    planning_ok_ = (move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!planning_ok_)
    {
      RCLCPP_ERROR(node_->get_logger(), "플래닝 실패");
      return BT::NodeStatus::FAILURE;
    }

    const auto *jmg = move_group_->getRobotModel()->getJointModelGroup("manipulator");
    visual_tools_->publishTrajectoryLine(plan_.trajectory_, jmg); // 시각화
    visual_tools_->trigger();

    auto exec_result = move_group_->execute(plan_);
    return (exec_result == moveit::core::MoveItErrorCode::SUCCESS) ? BT::NodeStatus::SUCCESS
                                                                   : BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    move_group_->stop();
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools>         visual_tools_;
  geometry_msgs::msg::PoseStamped                                 target_pose_;
  moveit::planning_interface::MoveGroupInterface::Plan            plan_;
  bool planning_ok_{false};
};





} // namespace strawberry_bt_nodes