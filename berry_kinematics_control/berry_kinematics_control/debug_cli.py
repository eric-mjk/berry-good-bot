# berry_kinematics_control/debug_cli.py
import rclpy, argparse, sys, time
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from berry_interface.action import MoveToNamedPose, MoveToPose, ServoTwist

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

def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd")

    p_named = sub.add_parser("named")
    p_named.add_argument("pose")

    p_pose = sub.add_parser("pose")
    p_pose.add_argument("xyzrpy", nargs=6, type=float)

    p_servo = sub.add_parser("servo")

    args = parser.parse_args()
    node = rclpy.create_node("berry_debug_cli")

    if args.cmd == "named":
        send_named(node, args.pose)
    elif args.cmd == "pose":
        send_pose(node, args.xyzrpy)
    else:
        start_servo(node)

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
# ros2 run berry_kinematics_control debug_cli pose 0.1 0.0 0.2 0 1.5708 0

# 3) Servo Twist 모드 시작
#    사용법: servo
#    이후 Ctrl-C 누르면 자동으로 액션 cancel
# ros2 run berry_kinematics_control debug_cli servo
