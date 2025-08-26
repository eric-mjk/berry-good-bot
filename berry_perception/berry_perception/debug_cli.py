#!/usr/bin/env python3
import rclpy, argparse, time
from rclpy.action import ActionClient
from berry_interface.action import DetectStrawberry, GripperControl, VisualServoDetect, VisualServoing

def main():
    rclpy.init()
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--offset", type=float, default=0.05,
    #                     help="camera Z-축에서 얼마나 뒤로 물러난 점을 반환할지[m]")
    # args = parser.parse_args()
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_det = sub.add_parser("detect", help="딸기 탐지 + 접근점 반환 (stage1/2 지원)")
    p_det.add_argument("--offset", type=float, default=0.05,
                       help="camera Z-축에서 얼마나 뒤로 물러날지[m]")
    p_det.add_argument("--stage", type=int, choices=[1,2], default=1,
                       help="1: 1차 rough approach, 2: 2차 rough approach")
    p_det.add_argument("--prev-x", type=float, default=0.0,
                       help="stage=2에서 사용할 이전 타겟 anchor X (base_link)")
    p_det.add_argument("--prev-y", type=float, default=0.0,
                       help="stage=2에서 사용할 이전 타겟 anchor Y (base_link)")
    p_det.add_argument("--prev-z", type=float, default=0.0,
                       help="stage=2에서 사용할 이전 타겟 anchor Z (base_link)")
    p_det.add_argument("--gate", type=float, default=0.12,
                       help="stage=2에서 anchor로부터 허용 반경[m] (0이면 무시)")

    p_grip = sub.add_parser("grip", help="그리퍼 열기/닫기")
    p_grip.add_argument("mode", choices=["open", "close"])

    # IBVS Visual Servoing (Action)
    p_ibvs = sub.add_parser("ibvs", help="IBVS visual servoing 시작/감시 (VisualServoing.action)")
    p_ibvs.add_argument("--timeout", type=float, default=15.0,
                        help="서보 최대 시간(초). 초과 시 서버가 실패 처리 (기본 15)")
    p_ibvs.add_argument("--duration", type=float, default=0.0, help="N초 후 자동 취소(0이면 Ctrl+C로 취소)")

    # Visual Servoing detector (streaming until cancel)
    p_vs = sub.add_parser("vs", help="실시간 YOLO 스트리밍 시작/취소 테스트 (visual_servo_detect)")
    p_vs.add_argument("--rgbd", action="store_true", help="RGBD 카메라 사용")
    p_vs.add_argument("--rgb",  action="store_true", help="USB RGB 카메라 사용(OpenCV)")
    p_vs.add_argument("--hz", type=float, default=10.0, help="카메라당 목표 Hz (기본 10)")
    p_vs.add_argument("--rgb-index", type=int, default=0, help="OpenCV RGB 카메라 인덱스")
    p_vs.add_argument("--duration", type=float, default=0.0,
                      help="N초 후 자동 취소(0이면 Ctrl+C로 취소)")
 
    args = parser.parse_args()

    node = rclpy.create_node("berry_debug_cli")

    if args.cmd == "detect":
        ac = ActionClient(node, DetectStrawberry, "detect_strawberry")
        node.get_logger().info("[CLI] waiting for detect_strawberry …")
        ac.wait_for_server()

        goal = DetectStrawberry.Goal()
        goal.offset_z = float(args.offset)
        # 새 필드들 세팅 (stage 1/2 분기용)
        goal.approach_stage = int(args.stage)
        goal.prev_target_base.x = float(args.prev_x)
        goal.prev_target_base.y = float(args.prev_y)
        goal.prev_target_base.z = float(args.prev_z)
        goal.prev_gate_m = float(args.gate)

        node.get_logger().info(f"[CLI] send goal: offset={goal.offset_z:.3f}, "
                               f"stage={goal.approach_stage}, "
                               f"prev=({goal.prev_target_base.x:.3f},"
                               f"{goal.prev_target_base.y:.3f},"
                               f"{goal.prev_target_base.z:.3f}), "
                               f"gate={goal.prev_gate_m:.3f}")

        gh_fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, gh_fut)
        gh = gh_fut.result()
        if not gh.accepted:
            node.get_logger().error("[CLI] detect_strawberry goal REJECTED")
            rclpy.shutdown()
            return

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut)
        result_msg = res_fut.result().result

        node.get_logger().info("[CLI] approach_pose_base →")
        node.get_logger().info(f"  pos = ({result_msg.approach_pose_base.pose.position.x:.3f}, "
                               f"{result_msg.approach_pose_base.pose.position.y:.3f}, "
                               f"{result_msg.approach_pose_base.pose.position.z:.3f})")
        node.get_logger().info(f"  ori = ({result_msg.approach_pose_base.pose.orientation.x:.3f}, "
                               f"{result_msg.approach_pose_base.pose.orientation.y:.3f}, "
                               f"{result_msg.approach_pose_base.pose.orientation.z:.3f}, "
                               f"{result_msg.approach_pose_base.pose.orientation.w:.3f})")
        try:
            tc = result_msg.strawberry_center_base
            node.get_logger().info(f"[CLI] strawberry_center_base = ({tc.x:.3f}, {tc.y:.3f}, {tc.z:.3f})")
        except Exception:
            node.get_logger().warn("[CLI] result: strawberry_center_base not available in action result")

    elif args.cmd == "grip":
        want_open = (args.mode == "open")
        ac = ActionClient(node, GripperControl, "gripper_command")
        node.get_logger().info("[CLI] waiting for gripper_command …")
        ac.wait_for_server()
        goal = GripperControl.Goal(open=want_open)
        gh = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, gh)
        node.get_logger().info(f"[CLI] gripper {'OPEN' if want_open else 'CLOSE'} 요청 완료")

    elif args.cmd == "vs":
        ac = ActionClient(node, VisualServoDetect, "visual_servo_detect")
        node.get_logger().info("[CLI] waiting for visual_servo_detect …")
        ac.wait_for_server()
        goal = VisualServoDetect.Goal(
            use_rgbd=bool(args.rgbd),
            use_rgb=bool(args.rgb),
            hz=float(args.hz),
            rgb_cam_index=int(args.rgb_index)
        )
        if not goal.use_rgbd and not goal.use_rgb:
            node.get_logger().error("[CLI] 하나 이상 카메라를 선택하세요 (--rgbd / --rgb)")
            rclpy.shutdown()
            return
        gh = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, gh)
        gh_res = gh.result()
        if not gh_res.accepted:
            node.get_logger().error("[CLI] goal REJECTED")
            rclpy.shutdown()
            return
        node.get_logger().info("[CLI] ▶ 스트리밍 시작됨. 토픽을 구독해서 결과를 보세요.")
        node.get_logger().info("      (디텍션: /visual_servo/rgbd/detection, /visual_servo/rgb/detection)")
        node.get_logger().info("      (디버그이미지: /visual_servo/rgbd/debug_image, /visual_servo/rgb/debug_image)")
        node.get_logger().info("      종료하려면 Ctrl+C를 누르거나 --duration N 사용")
        try:
            if args.duration > 0.0:
                t0 = time.time()
                while rclpy.ok() and (time.time() - t0) < args.duration:
                    rclpy.spin_once(node, timeout_sec=0.2)
                node.get_logger().info("[CLI] duration 만료 → CANCEL 전송")
            else:
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.5)
        except KeyboardInterrupt:
            node.get_logger().info("[CLI] Ctrl+C 감지 → CANCEL 전송")
        finally:
            cancel_future = gh_res.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future)
            result_future = gh_res.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            try:
                msg = result_future.result().result.message
            except Exception:
                msg = "(no message)"
            node.get_logger().info(f"[CLI] visual_servo_detect 종료: {msg}")
 
    elif args.cmd == "ibvs":
        # VisualServoing 액션 호출
        ac = ActionClient(node, VisualServoing, "visual_servoing")
        node.get_logger().info("[CLI] waiting for visual_servoing …")
        ac.wait_for_server()

        goal = VisualServoing.Goal(timeout_sec=float(args.timeout))

        def fb_cb(fb):
            try:
                f = fb.feedback
                node.get_logger().info(f"[IBVS fb] t={f.time_elapsed:.2f}s  state={f.state}")
            except Exception:
                pass

        send_future = ac.send_goal_async(goal, feedback_callback=fb_cb)
        rclpy.spin_until_future_complete(node, send_future)
        gh = send_future.result()
        if not gh.accepted:
            node.get_logger().error("[CLI] visual_servoing goal REJECTED")
            rclpy.shutdown()
            return

        node.get_logger().info(f"[CLI] ▶ IBVS 시작 (timeout={args.timeout:.2f}s). 종료하려면 Ctrl+C 또는 --duration 사용.")
        res_future = gh.get_result_async()
        try:
            if args.duration > 0.0:
                t0 = time.time()
                while rclpy.ok() and (time.time() - t0) < args.duration:
                    rclpy.spin_once(node, timeout_sec=0.2)
                node.get_logger().info("[CLI] duration 만료 → CANCEL 전송")
                cancel_future = gh.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future)
            else:
                # 결과가 올 때까지 대기 (Ctrl+C로 취소 가능)
                while rclpy.ok() and not res_future.done():
                    rclpy.spin_once(node, timeout_sec=0.5)
        except KeyboardInterrupt:
            node.get_logger().info("[CLI] Ctrl+C 감지 → CANCEL 전송")
            cancel_future = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future)

        # 최종 결과 수신/로그
        rclpy.spin_until_future_complete(node, res_future)
        result = res_future.result().result
        # 기본 종료 로그
        node.get_logger().info(
            f"[CLI] visual_servoing 종료: success={result.success}  message='{result.message}'"
        )
        # (있으면) 과일 클래스 집계 결과 출력
        fruit_cls = getattr(result, "fruit_class", "")
        fruit_sum = getattr(result, "fruit_class_conf_sum", 0.0)
        if fruit_cls or fruit_sum:
            try:
                node.get_logger().info(
                    f"[CLI] fruit_class='{fruit_cls}'  conf_sum={float(fruit_sum):.3f}"
                )
            except Exception:
                node.get_logger().info(
                    f"[CLI] fruit_class='{fruit_cls}'  conf_sum={fruit_sum}"
                )

    rclpy.shutdown()
if __name__ == "__main__":
    main()

# 사용 예:
# # 딸기 탐지 (stage1: conf≥threshold 후보 중 EEF 최근접)
# ros2 run berry_perception debug_cli detect --offset 0.05 --stage 1
# # 딸기 탐지 (stage2: 이전 타겟 기준 최근접 + 게이팅 0.12m)
# ros2 run berry_perception debug_cli detect --offset 0.01 --stage 2 --prev-x 0.35 --prev-y 0.05 --prev-z 0.18 --gate 0.12


# # 그리퍼 닫기
# ros2 run berry_perception debug_cli grip close
# ros2 run berry_perception debug_cli grip open


# --- Visual Servoing Detector 테스트 ---
# 1) RGBD만 10 Hz 스트리밍
# ros2 run berry_perception debug_cli vs --rgbd --hz 10
#
# 2) RGB(OpenCV)만 10 Hz, 인덱스 0
# ros2 run berry_perception debug_cli vs --rgb --rgb-index 0 --hz 10
#
# 3) 두 카메라 동시(교차 실행) 10 Hz, 5초 후 자동 취소
# ros2 run berry_perception debug_cli vs --rgbd --rgb --hz 10 --duration 5
#
# (실시간 결과 토픽)
#   /visual_servo/rgbd/detection, /visual_servo/rgb/detection
#   /visual_servo/rgbd/debug_image, /visual_servo/rgb/debug_image


# --- IBVS Visual Servoing 테스트 ---
# 1) 15초 타임아웃으로 실행
# ros2 run berry_perception debug_cli ibvs --timeout 15
#
# 2) 8초 후 자동 취소
# ros2 run berry_perception debug_cli ibvs --timeout 30 --duration 8
#
# 3) 수동 취소(Ctrl+C)
# ros2 run berry_perception debug_cli ibvs --timeout 30
# (서버 피드백은 t, state로 출력됨. 정상 종료/타임아웃/취소 모두 최종 메세지 로깅)d

 