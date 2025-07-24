#!/usr/bin/env python3
import rclpy, yaml, time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from builtin_interfaces.msg import Duration
from moveit.planning import MoveItPy                       # MoveIt2 Python API:contentReference[oaicite:3]{index=3}
from berry_interface.action import (
    MoveToPose, MoveToNamedPose, ServoTwist)
from rclpy.action import ActionServer, CancelResponse, GoalResponse

class BerryController(Node):
    def __init__(self):
        super().__init__("berry_controller")
        # MoveIt 초기화
        self.moveit = MoveItPy(node_name=self.get_name())
        self.arm = self.moveit.get_planning_component("manipulator")
        self.timer = self.create_timer(0.01, lambda: None)  # executor spin 유지

        # named pose 로드
        pose_path = str(
            self.get_package_share_directory("berry_good_bot_control")
        ) + "/config/named_poses.yaml"
        self.named = yaml.safe_load(open(pose_path, "r"))

        # Action 서버 3개
        self.pose_as = ActionServer(self, MoveToPose, "move_to_pose",
                                    self.execute_pose_cb)
        self.named_as = ActionServer(self, MoveToNamedPose, "move_to_named_pose",
                                     self.execute_named_cb)
        self.servo_as = ActionServer(self, ServoTwist, "servo_twist",
                                     self.execute_servo_cb)

        # Servo 모드용 Twist 구독
        self.twist_msg = None
        self.create_subscription(TwistStamped, "eef_twist",
                                 self.twist_cb, 10)

    # ---------- 콜백 ----------
    def execute_pose_cb(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info("Mode1: EEF pose로 이동")
        self.arm.clear()
        self.arm.set_goal_state(self.arm.get_current_state())
        self.arm.set_goal_pose(goal.target_pose, "tool0")
        plan_result = self.arm.plan()
        if not plan_result:
            goal_handle.abort()
            return MoveToPose.Result(success=False, message="Planning failed")
        self.arm.execute()
        goal_handle.succeed()
        return MoveToPose.Result(success=True, message="Done!")

    def execute_named_cb(self, goal_handle):
        name = goal_handle.request.pose_name
        if name not in self.named:
            goal_handle.abort()
            return MoveToNamedPose.Result(success=False,
                                          message=f"{name} not in YAML")
        self.get_logger().info(f"Mode2: Named pose {name} 이동")
        self.arm.clear()
        self.arm.set_goal_state(
            joint_positions=self.named[name],
            joint_names=["screw_joint","joint1","joint2","joint3","wrist_roll_joint"]
        )
        if not self.arm.plan():
            goal_handle.abort()
            return MoveToNamedPose.Result(success=False, message="Plan fail")
        self.arm.execute()
        goal_handle.succeed()
        return MoveToNamedPose.Result(success=True, message="Done!")

    def execute_servo_cb(self, goal_handle):
        dur = goal_handle.request.duration
        self.get_logger().info(f"Mode3: Servoing for {dur.sec} s")
        start = self.get_clock().now()
        r = self.create_rate(100)
        while (self.get_clock().now() - start) < rclpy.time.Time(seconds=dur.sec,
                                                                 nanoseconds=dur.nanosec):
            if self.twist_msg:
                # Servo 토픽은 이미 /servo_node 로 전달됨 (moveit_servo):contentReference[oaicite:4]{index=4}
                pass
            r.sleep()
        goal_handle.succeed()
        return ServoTwist.Result(success=True, message="Servo done")

    def twist_cb(self, msg: TwistStamped):
        self.twist_msg = msg

def main():
    rclpy.init()
    rclpy.spin(BerryController())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
