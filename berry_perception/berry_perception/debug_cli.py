#!/usr/bin/env python3
import rclpy, argparse
from rclpy.action import ActionClient
from berry_interface.action import DetectStrawberry

def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("--offset", type=float, default=0.05,
                        help="camera Z-축에서 얼마나 뒤로 물러난 점을 반환할지[m]")
    args = parser.parse_args()

    node = rclpy.create_node("perception_cli")
    ac   = ActionClient(node, DetectStrawberry, "detect_strawberry")
    ac.wait_for_server()

    goal = DetectStrawberry.Goal(offset_z=args.offset)
    gh_fut = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, gh_fut)
    res_fut = gh_fut.result().get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)

    result = res_fut.result().result
    print("Approach pose in base_link:\n", result.approach_pose_base)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

# 새 터미널:
# ros2 run berry_perception debug_cli --offset 0.05
#     RViz에서 /strawberry_mark(녹색)과 /approach_mark(빨간색)을 확인.
#     /tf 트리에서 base_link → … → link5 → camera_link 연결이 생겼는지 확인.
#     값이 어긋나면 camera_to_eef 파라미터만 바꿔가며 미세조정하세요.