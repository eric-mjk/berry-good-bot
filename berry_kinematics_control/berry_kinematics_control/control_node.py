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
        # self.declare_parameter("urdf_path")
        # self.declare_parameter("pose_yaml")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("pose_yaml", "")
        urdf_path  = self.get_parameter("urdf_path").get_parameter_value().string_value
        pose_yaml  = self.get_parameter("pose_yaml").get_parameter_value().string_value

        self.robot, self.ik_chain, self.joint_names = build_models(urdf_path)
        self.named_poses = load_named_poses(pose_yaml)
        self.n_joints = len(self.joint_names)
        # self.current_q = np.zeros(self.n_joints)

        self.n_joints = len(self.joint_names)

        # ① YAML 에 home 이 있으면 그 값을 첫 상태로 사용
        if "home" in self.named_poses:
            home = np.asarray(self.named_poses["home"], dtype=float)
            self.current_q = np.pad(home, (0, self.n_joints - len(home)), "constant")
        else:
            self.current_q = np.zeros(self.n_joints)   # fallback

        # ── 관절 한계 (origin 제외) ─────────────────────────
        self.q_bounds = np.array([lnk.bounds            # shape = (n, 2)
                                  for lnk in self.ik_chain.links[1:]])

        # joint_state 퍼블리셔
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_state_msg = JointState(name=self.joint_names)

        # 🔹 최초 1회 & 주기적으로 현재 값 송신 -----------------------
        self.publish_joint_state()               # 한 번 찍고
        self.idle_timer = self.create_timer(0.05,  # 20 Hz
                                            self.publish_joint_state)
        # 액션 서버 3종
        self.named_srv = ActionServer(
            self, MoveToNamedPose, "move_to_named_pose",
            execute_callback=self.exec_named)

        self.pose_srv = ActionServer(
            self, MoveToPose, "move_to_pose",
            execute_callback=self.exec_pose)

        self.servo_srv = ActionServer(
            self, ServoTwist, "servo_twist",
            execute_callback=self.exec_servo,
            goal_callback=self.servo_goal_cb,
            cancel_callback=self.servo_cancel_cb)

        # servo 모드 변수
        self.servo_active = False
        self._servo_busy  = False          # goal 수락 여부 제어
        self._servo_canceled = False       # 이번 goal이 취소됐는지 플래그
        self.latest_twist = np.zeros(6)
        self.twist_sub = self.create_subscription(
            Twist, "/eef_twist_cmd", self.twist_cb, 10)

        # servo 루프 타이머 (250 Hz)
        self.servo_timer = self.create_timer(0.04, self.servo_loop)

    # ---------- 공통 퍼블리시 ----------
    def publish_joint_state(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = self.current_q.tolist()
        self.js_pub.publish(self.joint_state_msg)
        self.get_logger().debug(f"publish_joint_state → q = {np.round(self.current_q,3).tolist()}")

    # ---------- Twist 콜백 ----------
    def twist_cb(self, msg: Twist):
        self.latest_twist = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z])

    # ---------- NamedPose ----------
    async def exec_named(self, goal_handle):
        pose_name = goal_handle.request.pose_name

        # self.get_logger().info(f"[exec_named] 요청받음 → `{pose_name}`")+        
        t0 = time.time()
        self.get_logger().info(f"[exec_named] ▶ START  pose=`{pose_name}`")

        if pose_name not in self.named_poses:
            goal_handle.abort()
            return MoveToNamedPose.Result(success=False,
                                          message="Unknown pose",
                                          )

        # target_q = np.asarray(self.named_poses[pose_name], dtype=float)
        target_q = np.asarray(self.named_poses[pose_name], dtype=float)
        # YAML 관절 개수가 실제보다 적으면 나머지는 현재 값 유지
        if target_q.size < self.n_joints:
            pad = self.current_q[target_q.size : self.n_joints]
            target_q = np.concatenate([target_q, pad])
        # 많으면 잘라내기
        elif target_q.size > self.n_joints:
            target_q = target_q[: self.n_joints]
        n_steps = 100
        for i, q in enumerate(lspb_interpolate(self.current_q, target_q, n_steps)):
            self.current_q = q
            self.publish_joint_state()
            fb = MoveToNamedPose.Feedback(progress=i/float(n_steps))
            goal_handle.publish_feedback(fb)
            time.sleep(0.06)                 # 10 ms 블로킹 (멀티스레드 Executor라 OK)
 
        err = float(np.linalg.norm(self.current_q - target_q))
        dt  = time.time() - t0
        self.get_logger().info(
            f"[exec_named] ✔ DONE  pose=`{pose_name}`  steps={n_steps}  "
            f"final_err={err:.6f}  elapsed={dt:.2f}s")

        goal_handle.succeed()
        return MoveToNamedPose.Result(success=True, message="Reached")

    # ---------- MoveToPose ----------
    async def exec_pose(self, goal_handle):
        target = goal_handle.request.target_pose

        # self.get_logger().info(f"[exec_pose] 요청받음 → pos=({target.pose.position.x:.3f},"
        t0 = time.time()
        self.get_logger().info(f"[exec_pose] ▶ START  pos=({target.pose.position.x:.3f},"
                               f"{target.pose.position.y:.3f},{target.pose.position.z:.3f}), "
                               "ori=(%.2f,%.2f,%.2f,%.2f)" % (
                                 target.pose.orientation.w,
                                 target.pose.orientation.x,
                                 target.pose.orientation.y,
                                 target.pose.orientation.z))

        # T = self.pose_to_matrix(target)
        # # ik_sol = self.ik_chain.inverse_kinematics(T, initial_position=np.r_[self.current_q, 0.0])
        
        # # ─── 여기가 핵심 ───
        # # IKPy 체인이 기대하는 길이만큼 0으로 패딩한 뒤
        # # 앞쪽 self.n_joints 칸에 current_q를 채웁니다.
        # mask     = self.ik_chain.active_links_mask     # 길이 8
        # init_q   = np.zeros(len(mask))                 # ✅ 8
        # self.get_logger().info(f"[exec_pose] len(mask) → ({len(mask)}")

        # j = 0
        # for i, active in enumerate(mask):
        #     if active:
        #         init_q[i] = self.current_q[j]
        #         j += 1

        # ik_sol = self.ik_chain.inverse_kinematics_frame(
        #     T,
        #     initial_position=init_q)

        # # ② 베이스(0)·툴0(마지막) 제외 → 딱 6칸
        # # target_q = np.asarray(ik_sol[1 : 1 + self.n_joints])
        # target_q = np.asarray(ik_sol[ mask ])          # 길이 6
        T = self.pose_to_matrix(target)                 # 4×4 목표 frame

        mask   = self.ik_chain.active_links_mask        # [False, True…]
        init_q = np.zeros(len(mask))                    # 패딩용 벡터

        # 현재 관절값을 active 슬롯에만 복사
        act_i = 0
        for i, active in enumerate(mask):
            if active:
                init_q[i] = self.current_q[act_i]
                act_i    += 1

        # IKPy : frame → joint
        # ik_sol = self.ik_chain.inverse_kinematics_frame(
        #     T, initial_position=init_q)

        # IKPy : frame → joint
        #   초기값이 joint limit 밖이면 SciPy 가 ValueError 를 던짐
        bounds = np.array([lnk.bounds for lnk in self.ik_chain.links])
        init_q = np.clip(init_q, bounds[:, 0] + 1e-4, bounds[:, 1] - 1e-4)

        ik_sol = self.ik_chain.inverse_kinematics_frame(
            T, initial_position=init_q)

        # origin(0) 제외하고 실제 관절만 추출
        target_q = np.asarray(ik_sol[1 : 1 + self.n_joints])
        n_steps = 150
        for i, q in enumerate(lspb_interpolate(self.current_q, target_q, n_steps)):
            self.current_q = q
            self.publish_joint_state()
            fb = MoveToPose.Feedback(progress=i/float(n_steps))
            goal_handle.publish_feedback(fb)
            time.sleep(0.06)

        err = float(np.linalg.norm(self.current_q - target_q))
        dt  = time.time() - t0
        self.get_logger().info(
            f"[exec_pose] ✔ DONE  steps={n_steps}  final_err={err:.6f}  elapsed={dt:.2f}s")
        goal_handle.succeed()
        return MoveToPose.Result(success=True, message="Reached")

    # ---------- ServoTwist ----------
    async def exec_servo(self, goal_handle):
        self.get_logger().info("[exec_servo] ▶ START  (entering servo mode)")

        # 이번 goal 시작 세팅
        self._servo_canceled = False
        self.servo_active = True
        try:
            # while rclpy.ok():
            #     # 클라이언트에서 cancel 요청되면 즉시 중단
            #     if goal_handle.is_cancel_requested:
            #         self.get_logger().info("[exec_servo] ⛔ CANCEL requested → stopping servo")
            #         self.servo_active = False
            #         self.latest_twist[:] = 0.0
            #         goal_handle.canceled()
            #         self.get_logger().info("[exec_servo] ✖ CANCELED")
            #         return ServoTwist.Result(success=False, message="Canceled")
            #     if not self.servo_active:
            #         break
            #     time.sleep(0.1)    # 100 ms 간격으로 상태 확인
            while rclpy.ok() and self.servo_active:
                # 취소가 서버 측에 반영되면 is_cancel_requested가 True가 됨
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("[exec_servo] ⛔ CANCEL requested → stopping servo")
                    self._servo_canceled = True
                    self.servo_active = False
                    break
                time.sleep(0.1)    # 100 ms 간격으로 상태 확인
        finally:
            # 루프 종료 시엔 최신 twist를 0으로 (안전)
            self.latest_twist[:] = 0.0
        # self.get_logger().info("[exec_servo] ✔ DONE  (servo stopped)")
        # goal_handle.succeed()
        # return ServoTwist.Result(success=True, message="Servo stopped")
            self._servo_busy = False

        # 결과 마킹
        if self._servo_canceled:
            self.get_logger().info("[exec_servo] ✖ CANCELED")
            goal_handle.canceled()
            return ServoTwist.Result(success=False, message="Canceled")
        else:
            self.get_logger().info("[exec_servo] ✔ DONE  (servo stopped)")
            goal_handle.succeed()
            return ServoTwist.Result(success=True, message="Servo stopped")

    def servo_loop(self):
        if not self.servo_active:
            return
        J = self.robot.jacob0(self.current_q)
        qdot = damped_pinv(J, lam=0.05) @ self.latest_twist
        self.current_q += qdot * 0.004

        # ★ 한계 클리핑 (DLS 초기화와 동일 오프셋) :cite:`turn0search8`
        eps = 1e-4
        self.current_q = np.clip(
            self.current_q,
            self.q_bounds[:, 0] + eps,
            self.q_bounds[:, 1] - eps)

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

    # ---------- 액션 콜백들 ----------
    def servo_goal_cb(self, goal_request):
        """
        동시에 하나의 서보 goal만 허용. 바쁘면 REJECT.
        """
        if self._servo_busy or self.servo_active:
            self.get_logger().warn("[servo_goal_cb] busy → reject new servo goal")
            return GoalResponse.REJECT
        self._servo_busy = True
        self.get_logger().info("[servo_goal_cb] ACCEPT servo goal")
        return GoalResponse.ACCEPT

    def servo_cancel_cb(self, cancel_request):
        """
        취소 요청은 항상 승인. 즉시 루프를 멈추도록 플래그/트위스트 0 설정.
        (rclpy 기본은 'No cancellations' 이라 명시적으로 ACCEPT 해야 함)
        """
        self.get_logger().info("[servo_cancel_cb] CANCEL request received → ACCEPT")
        self._servo_canceled = True
        self.servo_active = False
        self.latest_twist[:] = 0.0
        return CancelResponse.ACCEPT
    
def main():
    # rclpy.init()
    # node = BerryControl()
    # rclpy.spin(node)

    import rclpy.executors as execs

    rclpy.init()
    node = BerryControl()

    executor = execs.MultiThreadedExecutor(num_threads=4)   # 🔹 멀티스레드
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()