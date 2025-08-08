#!/usr/bin/env python3
import rclpy, argparse
from rclpy.action import ActionClient
from berry_interface.action import DetectStrawberry, GripperControl

def main():
    rclpy.init()
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--offset", type=float, default=0.05,
    #                     help="camera Z-축에서 얼마나 뒤로 물러난 점을 반환할지[m]")
    # args = parser.parse_args()
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_det = sub.add_parser("detect", help="딸기 탐지 + 접근점 반환")
    p_det.add_argument("--offset", type=float, default=0.05,
                       help="camera Z-축에서 얼마나 뒤로 물러날지[m]")

    p_grip = sub.add_parser("grip", help="그리퍼 열기/닫기")
    p_grip.add_argument("mode", choices=["open", "close"])

    args = parser.parse_args()

    node = rclpy.create_node("berry_debug_cli")

    if args.cmd == "detect":
        ac = ActionClient(node, DetectStrawberry, "detect_strawberry")
        node.get_logger().info("[CLI] waiting for detect_strawberry …")
        ac.wait_for_server()
        goal = DetectStrawberry.Goal(offset_z=args.offset)
        gh = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, gh)
        res = gh.result().get_result_async()
        rclpy.spin_until_future_complete(node, res)
        node.get_logger().info(f"[CLI] approach_pose_base →\n{res.result().result.approach_pose_base}")

    elif args.cmd == "grip":
        want_open = (args.mode == "open")
        ac = ActionClient(node, GripperControl, "gripper_command")
        node.get_logger().info("[CLI] waiting for gripper_command …")
        ac.wait_for_server()
        goal = GripperControl.Goal(open=want_open)
        gh = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, gh)
        node.get_logger().info(f"[CLI] gripper {'OPEN' if want_open else 'CLOSE'} 요청 완료")


    rclpy.shutdown()
if __name__ == "__main__":
    main()

# 사용 예:
# # 딸기 탐지
# ros2 run berry_perception debug_cli detect --offset 0.05
# # 그리퍼 닫기
# ros2 run berry_perception debug_cli grip close
# ros2 run berry_perception debug_cli grip open
