#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  // 1) ROS 초기화
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<rclcpp::Node>("task_planner", opts);

  // 2) BT XML 파일 경로 파라미터
  std::string xml_file =
    node->declare_parameter("bt_tree", "/tmp/harvest.xml");

  // 3) BehaviorTreeFactory 선언 및 ROS 플러그인 자동 로드
  BT::BehaviorTreeFactory factory;
  factory.registerFromROSPlugins();  
  //  이 한 줄로 워크스페이스 내 모든 BT 플러그인(.so)에 대해
  //  BT_REGISTER_NODES를 호출합니다 :contentReference[oaicite:1]{index=1}

  // 4) 트리 생성
  auto tree = factory.createTreeFromFile(xml_file);

  // 5) 로깅 설정 (v3 전용)
  BT::StdCoutLogger logger(tree);     // 콘솔에 노드 상태 변화 출력
  BT::PublisherZMQ  zmq_pub(tree);    // Groot GUI 실시간 시각화

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
