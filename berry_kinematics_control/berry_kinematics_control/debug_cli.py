# berry_kinematics_control/debug_cli.py
# pip install pygame
import rclpy, argparse, sys, time
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from berry_interface.action import MoveToNamedPose, MoveToPose, ServoTwist
# from pynput import keyboard        # ← 새로
import os, pygame                  # ← pygame 사용
import numpy as np

def send_named(node, pose):
    ac = ActionClient(node, MoveToNamedPose, "move_to_named_pose")
    ac.wait_for_server()
    goal = MoveToNamedPose.Goal(pose_name=pose)
    # fut = ac.send_goal_async(goal)
    # rclpy.spin_until_future_complete(node, fut)
    # print("NamedPose result:", fut.result().result)
    gh_fut = ac.send_goal_async(goal)                 # ① GoalHandle future
    rclpy.spin_until_future_complete(node, gh_fut)
    gh     = gh_fut.result()
    res_fut = gh.get_result_async()                   # ② Result future
    rclpy.spin_until_future_complete(node, res_fut)
    print("NamedPose result:", res_fut.result().result)

def send_pose(node, xyzrpy):
    ac = ActionClient(node, MoveToPose, "move_to_pose")
    ac.wait_for_server()
    pose = PoseStamped()
    pose.header.frame_id = "tool0"
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyzrpy[:3]
    import transforms3d as t3d
    quat = t3d.euler.euler2quat(*xyzrpy[3:])
    pose.pose.orientation.w, pose.pose.orientation.x, \
        pose.pose.orientation.y, pose.pose.orientation.z = quat
    goal = MoveToPose.Goal(target_pose=pose)
    # fut = ac.send_goal_async(goal)
    # rclpy.spin_until_future_complete(node, fut)
    # print("MoveToPose result:", fut.result().result)
    gh_fut = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, gh_fut)
    gh     = gh_fut.result()
    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    print("MoveToPose result:", res_fut.result().result)


def start_servo(node):
    ac = ActionClient(node, ServoTwist, "servo_twist")
    ac.wait_for_server()
    goal = ServoTwist.Goal()
    fut = ac.send_goal_async(goal)
    pub = node.create_publisher(Twist, "/eef_twist_cmd", 10)
    print("Streaming random twist (Ctrl-C to stop)…")
    try:
        while rclpy.ok():
            tw = Twist()
            tw.linear.x = 0.02
            pub.publish(tw)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    ac.cancel_goal_async(fut.result().goal_id)

# ---------- 키보드 서보 ----------
def keyboard_servo(node):
    # --- ROS 액션 시작 -----------------------------------------------------
    ac = ActionClient(node, ServoTwist, "servo_twist")
    ac.wait_for_server()
    gh_fut = ac.send_goal_async(ServoTwist.Goal())
    rclpy.spin_until_future_complete(node, gh_fut)

    pub = node.create_publisher(Twist, "/eef_twist_cmd", 10)

    # --- pygame 초기화 -----------------------------------------------------
    os.environ.setdefault("SDL_VIDEO_CENTERED", "1")
    pygame.init()
    screen = pygame.display.set_mode((480, 120))
    pygame.display.set_caption("BerryBot Tele-op  (ESC to quit)")
    font = pygame.font.SysFont(None, 20)

    RATE = 50        # [Hz]
    VLIN = 0.1
    VANG = 0.5

    key_map = {
        pygame.K_w: (1, +VLIN), pygame.K_s: (1, -VLIN),
        pygame.K_a: (0, -VLIN), pygame.K_d: (0, +VLIN),
        pygame.K_e: (2, +VLIN), pygame.K_q: (2, -VLIN),
        pygame.K_u: (3, +VANG), pygame.K_o: (3, -VANG),
        pygame.K_i: (4, +VANG), pygame.K_k: (4, -VANG),
        pygame.K_j: (5, +VANG), pygame.K_l: (5, -VANG),
    }
    twist_vec = np.zeros(6)
    clock = pygame.time.Clock()
    print("▶ 키보드 창(포커스 필요)에서 조작하세요. ESC 종료")
    running = True
    while rclpy.ok() and running:
        # ── 이벤트 처리 ───────────────────────────────────────────
        for evt in pygame.event.get():
            if evt.type == pygame.QUIT:
                running = False
            elif evt.type in (pygame.KEYDOWN, pygame.KEYUP):
                press = (evt.type == pygame.KEYDOWN)
                if evt.key == pygame.K_ESCAPE:
                    running = False
                elif evt.key in key_map:
                    axis, step = key_map[evt.key]
                    twist_vec[axis] = step if press else 0.0

        # ── Twist 발행 ────────────────────────────────────────────
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = twist_vec[:3]
        tw.angular.x, tw.angular.y, tw.angular.z = twist_vec[3:]
        pub.publish(tw)

        # 작은 상태표시
        screen.fill((30, 30, 30))
        txt = font.render(
              f"vx={tw.linear.x:+.2f}  vy={tw.linear.y:+.2f}  vz={tw.linear.z:+.2f}  |  "
              f"wx={tw.angular.x:+.2f}  wy={tw.angular.y:+.2f}  wz={tw.angular.z:+.2f}",
              True, (200, 200, 200))
        screen.blit(txt, (10, 20))
        pygame.display.flip()

        clock.tick(RATE)

    # --- 정리 --------------------------------------------------------------
    pygame.quit()
    # ac.cancel_goal_async(gh_fut.result().goal_id)

    goal_handle = gh_fut.result()                 # 🔹 ClientGoalHandle
    cancel_fut  = goal_handle.cancel_goal_async() # 메서드는 여기 존재
    rclpy.spin_until_future_complete(node, cancel_fut)

def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd")

    p_named = sub.add_parser("named")
    p_named.add_argument("pose")

    p_pose = sub.add_parser("pose")
    p_pose.add_argument("xyzrpy", nargs=6, type=float)

    p_servo = sub.add_parser("servo")
    p_servo.add_argument("--random", action="store_true",
                         help="무작위 twist 대신 키보드 제어")

    args = parser.parse_args()
    node = rclpy.create_node("berry_debug_cli")

    if args.cmd == "named":
        send_named(node, args.pose)
    elif args.cmd == "pose":
        send_pose(node, args.xyzrpy)
    # else:
    #     start_servo(node)
    elif args.cmd == "servo" and args.random:
        start_servo(node)                # 기존 random 모드
    else:
        keyboard_servo(node)             # 🔹 새 키보드 서보 모드

    rclpy.shutdown()


# 사용법
# 1) Named Pose 보내기
#    사용법: named <pose_name>
#    예시: "home", "ready", "basket" 등의 YAML에 정의된 포즈 이름을 넣어주세요.
# ros2 run berry_kinematics_control debug_cli named home

# 2) 임의의 Cartesian 목표 자세 보내기
#    사용법: pose X Y Z ROLL PITCH YAW
#    (단위: X,Y,Z[m], ROLL/PITCH/YAW[rad])
#    예시: (0.1, 0.0, 0.2) 위치, (0, π/2, 0) 자세
# ros2 run berry_kinematics_control debug_cli pose 0.2958 0.4269 0.2035 1.5708 0.0 -2.9205

# 3) Servo Twist 모드 시작
#    사용법: servo
#    이후 Ctrl-C 누르면 자동으로 액션 cancel
# ros2 run berry_kinematics_control debug_cli servo
