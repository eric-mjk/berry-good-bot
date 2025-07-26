# berry_kinematics_control/control_node.py
import rclpy, numpy as np, time, threading
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node   import Node
from sensor_msgs.msg   import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from berry_interface.action import MoveToNamedPose, MoveToPose, ServoTwist

from .kin_utils import build_models, load_named_poses, damped_pinv, lspb_interpolate

class BerryControl(Node):

    def __init__(self):
        super().__init__("berry_control")

        # ---------------- 파라미터 ----------------
        self.declare_parameter("urdf_path")
        self.declare_parameter("pose_yaml")
        urdf_path  = self.get_parameter("urdf_path").get_parameter_value().string_value
        pose_yaml  = self.get_parameter("pose_yaml").get_parameter_value().string_value

        self.robot, self.ik_chain, self.joint_names = build_models(urdf_path)
        self.named_poses = load_named_poses(pose_yaml)
        self.n_joints = len(self.joint_names)
        self.current_q = np.zeros(self.n_joints)

        # joint_state 퍼블리셔
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_state_msg = JointState(name=self.joint_names)

        # 액션 서버 3종
        self.named_srv = ActionServer(
            self, MoveToNamedPose, "move_to_named_pose",
            execute_callback=self.exec_named)

        self.pose_srv = ActionServer(
            self, MoveToPose, "move_to_pose",
            execute_callback=self.exec_pose)

        self.servo_srv = ActionServer(
            self, ServoTwist, "servo_twist",
            execute_callback=self.exec_servo)

        # servo 모드 변수
        self.servo_active = False
        self.latest_twist = np.zeros(6)
        self.twist_sub = self.create_subscription(
            Twist, "/eef_twist_cmd", self.twist_cb, 10)

        # servo 루프 타이머 (250 Hz)
        self.servo_timer = self.create_timer(0.004, self.servo_loop)

    # ---------- 공통 퍼블리시 ----------
    def publish_joint_state(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = self.current_q.tolist()
        self.js_pub.publish(self.joint_state_msg)

    # ---------- Twist 콜백 ----------
    def twist_cb(self, msg: Twist):
        self.latest_twist = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z])

    # ---------- NamedPose ----------
    async def exec_named(self, goal_handle):
        pose_name = goal_handle.request.pose_name
        if pose_name not in self.named_poses:
            goal_handle.abort()
            return MoveToNamedPose.Result(success=False,
                                          message="Unknown pose",
                                          )

        target_q = np.asarray(self.named_poses[pose_name], dtype=float)
        for i, q in enumerate(lspb_interpolate(self.current_q, target_q, 100)):
            self.current_q = q
            self.publish_joint_state()
            fb = MoveToNamedPose.Feedback(progress=i/100.0)
            goal_handle.publish_feedback(fb)
            await rclpy.sleep(0.01)

        goal_handle.succeed()
        return MoveToNamedPose.Result(success=True, message="Reached")

    # ---------- MoveToPose ----------
    async def exec_pose(self, goal_handle):
        target = goal_handle.request.target_pose
        T = self.pose_to_matrix(target)
        ik_sol = self.ik_chain.inverse_kinematics(T, initial_position=np.r_[self.current_q, 0.0])
        target_q = np.asarray(ik_sol[:-1])  # IKPy appends dummy joint

        for i, q in enumerate(lspb_interpolate(self.current_q, target_q, 150)):
            self.current_q = q
            self.publish_joint_state()
            fb = MoveToPose.Feedback(progress=i/150.0)
            goal_handle.publish_feedback(fb)
            await rclpy.sleep(0.01)

        goal_handle.succeed()
        return MoveToPose.Result(success=True, message="Reached")

    # ---------- ServoTwist ----------
    async def exec_servo(self, goal_handle):
        self.servo_active = True
        while self.servo_active and rclpy.ok():
            await rclpy.sleep(0.1)
        goal_handle.succeed()
        return ServoTwist.Result(success=True, message="Servo stopped")

    def servo_loop(self):
        if not self.servo_active:
            return
        J = self.robot.jacob0(self.current_q)
        qdot = damped_pinv(J, lam=0.05) @ self.latest_twist
        self.current_q += qdot * 0.004
        self.publish_joint_state()

    # ---------- 유틸 ----------
    @staticmethod
    def pose_to_matrix(ps: PoseStamped):
        import transforms3d as t3d
        p = ps.pose.position
        o = ps.pose.orientation
        R = t3d.quaternions.quat2mat([o.w, o.x, o.y, o.z])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = [p.x, p.y, p.z]
        return T

def main():
    rclpy.init()
    node = BerryControl()
    rclpy.spin(node)
