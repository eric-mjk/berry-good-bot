//run_bt_node.cpp
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>

// 직접 포함할 노드 헤더 파일
// strawberry_task_planner의 include/strawberry_bt_nodes/ 디렉토리에서 가져옵니다.
#include "strawberry_task_planner/fake_nodes.hpp" 
#include "strawberry_task_planner/rough_approach_nodes.hpp" // 새로 추가할 노드

namespace strawberry_bt_nodes {
  rclcpp::Node::SharedPtr g_ros_node;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    auto node = std::make_shared<rclcpp::Node>("task_planner", opts);


    // --- 핵심 수정: 정적 ROS 노드 포인터 초기화 ---
    // 모든 Behavior Tree 노드들이 이 'node'를 공유하게 됩니다.
    strawberry_bt_nodes::g_ros_node = node;

    // IMPORTANT: Create a MultiThreadedExecutor to handle MoveIt's internal callbacks
    // MoveIt 2 typically requires a multi-threaded executor to function correctly,
    // as it subscribes to various topics and services.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // Start a separate thread to spin the executor
    std::thread spinner_thread([&executor]() {
        executor.spin();
    });

    // 2) BT XML 파일 경로 파라미터
    // 파라미터로 넘어오는 값이 이제 완전한 절대 경로입니다.
    std::string xml_file_absolute_path = node->declare_parameter<std::string>("bt_tree_file", ""); // 기본값을 빈 문자열로 설정

    if (xml_file_absolute_path.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No BT XML file path provided! Please set 'bt_tree_file' parameter.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Loading Behavior Tree from: %s", xml_file_absolute_path.c_str()); // 로깅 추가

    BT::BehaviorTreeFactory factory;


    // 3) Hydra 파라미터에 따라 노드를 직접 등록
    // 'register_node_types' 파라미터를 추가하여 어떤 노드 그룹을 등록할지 제어합니다.
    // 예: register_node_types: ["fake_nodes"]
    std::vector<std::string> node_types_to_register = node->declare_parameter<std::vector<std::string>>("register_node_types", std::vector<std::string>());

    // "fake_nodes" 그룹이 등록될 목록에 있다면, 해당 노드들을 등록
    for (const std::string& type_name : node_types_to_register) {
        if (type_name == "fake_nodes") {
            RCLCPP_INFO(node->get_logger(), "Registering Fake Nodes (StrawberryPerception, RoughApproach, ServoAndCut)");
            factory.registerNodeType<FakePerception>("fake_StrawberryPerception");
            factory.registerNodeType<FakeApproach>("fake_RoughApproach");
            factory.registerNodeType<FakeServoCut>("fake_ServoAndCut");
        } else if (type_name == "rough_approach_nodes") {

            RCLCPP_INFO(node->get_logger(), "Registering Rough Approach Nodes (TestPerception, PerceptionVisualizer, RoughApproach)");
            factory.registerNodeType<strawberry_bt_nodes::TestPerception>("TestPerception");
            factory.registerNodeType<strawberry_bt_nodes::PerceptionVisualizer>("PerceptionVisualizer");
            factory.registerNodeType<strawberry_bt_nodes::RoughApproach>("RoughApproach");

        } else {
                RCLCPP_WARN(node->get_logger(), "Unknown node type group '%s' in 'register_node_types' parameter. Skipping.", type_name.c_str());
            }
        }
    
    // 4) 트리 생성 - 넘어온 절대 경로를 그대로 사용합니다.
    auto tree = factory.createTreeFromFile(xml_file_absolute_path);

    // 5) 로깅 설정 (v3 전용)
    BT::StdCoutLogger logger(tree);
    BT::PublisherZMQ  zmq_pub(tree);

    // 6) 실행 루프 (1 Hz)
    rclcpp::Rate loop(1.0);
    while (rclcpp::ok()) {
      auto status = tree.tickRoot();
      if (status == BT::NodeStatus::SUCCESS ||
          status == BT::NodeStatus::FAILURE) {
        break;
      }
      loop.sleep();
    }

    // 7) 종료
    rclcpp::shutdown();
    return 0;
}