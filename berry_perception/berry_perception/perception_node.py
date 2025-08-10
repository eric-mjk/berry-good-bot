#!/usr/bin/env python3
import rclpy, math, cv2, torch, numpy as np, os, sys
from rclpy.node            import Node
from rclpy.action          import ActionServer
from sensor_msgs.msg       import Image, CameraInfo
from geometry_msgs.msg     import TransformStamped, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from tf2_ros               import StaticTransformBroadcaster, Buffer, TransformListener, TransformBroadcaster
import tf_transformations   as tft
from cv_bridge             import CvBridge
from ultralytics           import YOLO
from berry_interface.action import DetectStrawberry, GripperControl
from std_msgs.msg import Int8

from ament_index_python.packages import get_package_share_directory

class PerceptionNode(Node):
    def __init__(self):
        # super().__init__('perception_node')
        super().__init__(
            "perception_node",
            # YAML-override 에서 온 모든 키를 자동 선언
            automatically_declare_parameters_from_overrides=True
        )

        # ────── Logger ────────────────────────────────────────────
        self.logger = self.get_logger()
        self.logger.info(f"[INIT] argv → {sys.argv}")
 
        # ────── Parameters ───────────────────────────────────────────
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('eef_frame',     'link5')
        # self.declare_parameter('camera_to_eef', [0.0, 0.08, 0.045,  math.pi/180*30, -math.pi/2, -math.pi/2]) # xyz + rpy
        self.declare_parameter('camera_to_eef', [0.035, 0.09, 0.045,  math.pi/180*0, -math.pi/2, -math.pi/2]) # xyz + rpy
        cam2eef = self.get_parameter('camera_to_eef').value
        self.logger.debug(f"[PARAM] camera_to_eef = {cam2eef}")
        self.publish_static_tf(cam2eef)

        # ────── Subscribers / utils ─────────────────────────────────
        self.bridge   = CvBridge()
        self.rgb_sub  = self.create_subscription(Image,  '/camera/camera/color/image_raw', self.rgb_cb, 10)
        self.dep_sub  = self.create_subscription(Image,  '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)

        self.rgb_img   = None
        self.depth_img = None
        self.cam_info  = None

        # TF buffer for base_link 변환
        self.tfb  = Buffer()
        self.tfl  = TransformListener(self.tfb, self)
        # 접근 목표를 TF로도 시각화하기 위한 동적 브로드캐스터
        self.tf_dyn = TransformBroadcaster(self)

        # RViz marker publisher
        self.marker_pub = self.create_publisher(Marker, 'strawberry_mark',  10)
        self.appr_pub   = self.create_publisher(Marker, 'approach_mark',    10)

        # ────── YOLOv8 초기화 ───────────────────────────────────────
        pkg_share  = get_package_share_directory('berry_perception')
        model_path = os.path.join(pkg_share, 'model', 'model1.pt')
        self.logger.info(f"[YOLO] loading weights: {model_path}")
        self.model = YOLO(model_path)
        self.model.model.to('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.logger.info(f"[YOLO] device → {self.model.device}")

        # ────── Action Servers ──────────────────────────────────────
        self._action_server = ActionServer(
            self, DetectStrawberry, 'detect_strawberry', self.execute_cb)
        
        self._grip_server = ActionServer(                     # 🔸 추가
            self, GripperControl,  'gripper_command', self.gripper_cb)

        # ────── Gripper Command 토픽 (2 Hz) ───────────────────────
        self.grip_pub   = self.create_publisher(Int8, '/gripper_cmd', 10)
        self._grip_val  = 1                              # 기본 열림
        self.create_timer(0.5, self._tick_grip)         # 2 Hz

    # ----------------- Static TF ----------------------------------
    def publish_static_tf(self, xyzrpy):
        br = StaticTransformBroadcaster(self)
        t  = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('eef_frame').value
        t.child_frame_id  = self.get_parameter('camera_frame').value
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyzrpy[:3]
        # q = tft.quaternion_from_euler(*xyzrpy[3:])
        # t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # ── (1) 원래 RPY → 행렬 ────────────────────────────────
        base_rot = tft.euler_matrix(*xyzrpy[3:])          # 4×4

        # ── (2) 보정: 초록(Y)축 기준 +30° 앞쪽으로 기울이기 ──
        tilt_rad = math.radians(35.0)                     # 필요 시 −값으로 바꿔보세요
        corr_mat = tft.rotation_matrix(tilt_rad, (0, 1, 0))

        # ── (3) 두 행렬 곱 → 최종 쿼터니언 ────────────────────
        final_q  = tft.quaternion_from_matrix(
                      tft.concatenate_matrices(base_rot, corr_mat))
        t.transform.rotation = Quaternion(
           x=final_q[0], y=final_q[1], z=final_q[2], w=final_q[3])
        br.sendTransform(t)

    # ----------------- Subs ---------------------------------------
    # def rgb_cb(self, msg):   self.rgb_img   = msg
    # def depth_cb(self, msg): self.depth_img = msg
    # def info_cb(self, msg):  self.cam_info  = msg
    def rgb_cb(self, msg):
        self.rgb_img = msg
        if not hasattr(self, "_dbg_rgb_once"):
            self.logger.debug("[CB] first RGB frame received")
            self._dbg_rgb_once = True

    def depth_cb(self, msg):
        self.depth_img = msg
        if not hasattr(self, "_dbg_depth_once"):
            self.logger.debug("[CB] first Depth frame received")
            self._dbg_depth_once = True

    def info_cb(self, msg):
        self.cam_info = msg
        if not hasattr(self, "_dbg_info_once"):
            self.logger.debug("[CB] first CameraInfo received")
            self._dbg_info_once = True

    # ----------------- Main Action --------------------------------
    async def execute_cb(self, goal_handle):
        offset_z = goal_handle.request.offset_z
        self.logger.info(f"[ACTION] new goal: offset_z={offset_z:.3f}")

        # 최신 frame 올 때까지 대기
        while self.rgb_img is None or self.depth_img is None or self.cam_info is None:
            self.logger.info(f"[ACTION] no img!!")
            rclpy.spin_once(self, timeout_sec=0.1)

        cv_img = self.bridge.imgmsg_to_cv2(self.rgb_img, 'bgr8')
        results = self.model.predict(source=cv_img, conf=0.4, save=False, verbose=False)
        if not results or not results[0].boxes:
            self.get_logger().warn('No strawberry detected')
            goal_handle.abort()
            return DetectStrawberry.Result()

        # 가장 높은 confidence 선택
        boxes = results[0].boxes
        best  = boxes[boxes.conf.argmax()]
        self.logger.info(f"[YOLO] {len(boxes)} boxes, best conf={float(best.conf):.3f}")

        x1, y1, x2, y2 = map(int, best.xyxy[0])
        cx, cy = int((x1+x2)/2), int((y1+y2)/2)

        # z값(dept) 확보
        # depth = self.bridge.imgmsg_to_cv2(self.depth_img, 'passthrough')[cy, cx] / 1000.0  # mm→m
        # if depth == 0.0:
        #     goal_handle.abort()
        #     return DetectStrawberry.Result()

        # ─── 깊이 추정 (박스 전체 → 이상치 제거 후 robust median) ────
        depth_np = self.bridge.imgmsg_to_cv2(
            self.depth_img, 'passthrough').astype(np.float32) / 1000.0
        win = depth_np[y1:y2, x1:x2].reshape(-1)
        vals = win[(win > 0) & np.isfinite(win)]
        if vals.size == 0:
            self.logger.warn("No valid depth → abort")
            goal_handle.abort()
            return DetectStrawberry.Result()
        med  = np.median(vals)
        mad  = np.median(np.abs(vals - med))
        thresh = 3 * 1.4826 * mad if mad > 0 else np.inf
        inliers = vals[np.abs(vals - med) < thresh]
        # depth = float(np.median(inliers))            # robust depth  :contentReference[oaicite:0]{index=0}
        # ─── 깊이 추정 방식 변경: 하위 25 % 값들의 평균 ───
        sorted_vals = np.sort(inliers)
        q25_len = max(1, int(len(sorted_vals) * 0.25))
        depth = float(np.mean(sorted_vals[:q25_len]))
        self.logger.info(f"[DEPTH] {len(inliers)}/{len(vals)} inliers, depth={depth:.3f} m")


        self.logger.info(f"[DEPTH] depth({cx},{cy}) = {depth:.3f} m")

        # # 픽셀→3D (pin-hole)
        # fx = self.cam_info.k[0];  fy = self.cam_info.k[4]
        # cx0 = self.cam_info.k[2]; cy0 = self.cam_info.k[5]
        # X = (cx - cx0) * depth / fx
        # Y = (cy - cy0) * depth / fy
        # Z = depth
        # self.logger.info(f"[PINHOLE] cam-coords X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")
        # ─── ① Optical-Frame 좌표 (REP-103: x→right, y→down, z→forward)
        fx = self.cam_info.k[0];  fy = self.cam_info.k[4]
        cx0 = self.cam_info.k[2]; cy0 = self.cam_info.k[5]
        x_opt = (cx - cx0) * depth / fx
        y_opt = (cy - cy0) * depth / fy
        z_opt = depth

        # ─── ② Optical → Body( camera_link ) 변환
        #    Body-Frame : x→forward, y→left, z→up
        X =  z_opt          # forward
        Y = -x_opt          # left
        Z = -y_opt          # up
        self.logger.info(
            f"[PINHOLE] optical (x={x_opt:.3f}, y={y_opt:.3f}, z={z_opt:.3f}) → "
            f"body (X={X:.3f}, Y={Y:.3f}, Z={Z:.3f})")

        st_pose_cam = PoseStamped()
        st_pose_cam.header.stamp = self.rgb_img.header.stamp
        st_pose_cam.header.frame_id = self.get_parameter('camera_frame').value
        self.logger.info(f"[FRAME] frame_id = {st_pose_cam.header.frame_id}")
        st_pose_cam.pose.position = Point(x=X, y=Y, z=Z)
        st_pose_cam.pose.orientation.w = 1.0  # no orientation

        # feedback
        goal_handle.publish_feedback(
            DetectStrawberry.Feedback(strawberry_pose_cam=st_pose_cam))

        p_cam  = np.array([X, Y, Z, 1.0])
        try:
            tf_cam2base = self.tfb.lookup_transform(
                'base_link', st_pose_cam.header.frame_id,
                rclpy.time.Time())                      # 최신 TF
            mat_cam2base = tft.concatenate_matrices(
                tft.translation_matrix((tf_cam2base.transform.translation.x,
                                        tf_cam2base.transform.translation.y,
                                        tf_cam2base.transform.translation.z)),
                tft.quaternion_matrix(
                    (tf_cam2base.transform.rotation.x,
                     tf_cam2base.transform.rotation.y,
                     tf_cam2base.transform.rotation.z,
                     tf_cam2base.transform.rotation.w)))
        except Exception as e:
            self.get_logger().error(f"TF error: {e}")
            goal_handle.abort()
            return DetectStrawberry.Result()

        st_base = (mat_cam2base @ p_cam)[:3]            # 딸기 위치(base)

        # # ② link4 z축으로 0.04 m 이동
        # tf_link4 = self.tfb.lookup_transform('base_link', 'link4', rclpy.time.Time())
        # R_link4  = tft.quaternion_matrix((tf_link4.transform.rotation.x,
        #                                   tf_link4.transform.rotation.y,
        #                                   tf_link4.transform.rotation.z,
        #                                   tf_link4.transform.rotation.w))
        # z_axis   = R_link4[:3, 2]
        # appr_pos = st_base + 0.04 * z_axis

        # # ③ link5 원점 → 딸기 벡터 방향으로 offset_z 만큼 당겨오기
        # tf_link5  = self.tfb.lookup_transform('base_link', 'link5', rclpy.time.Time())
        # link5_org = np.array([tf_link5.transform.translation.x,
        #                       tf_link5.transform.translation.y,
        #                       tf_link5.transform.translation.z])
        # vec       = st_base - link5_org
        # vec_norm  = vec / np.linalg.norm(vec)
        # appr_pos  = appr_pos - offset_z * vec_norm

        # ② 접근점 계산 (필수: link4의 XY 평면으로 사영한 벡터 사용)
        #    - vec = (딸기 - link5 원점)
        #    - vec 을 link4의 XY 평면(법선 = link4 z축)으로 사영 → v_proj
        #    - 최종 접근점 = st_base + 0.04 * z4  -  offset_z * normalize(v_proj)
        #
        #    주의: 사영 벡터 크기가 너무 작으면(link4 z축과 거의 평행) link4 x축을 대체 방향으로 사용
        tf_link4 = self.tfb.lookup_transform('base_link', 'link4', rclpy.time.Time())
        R_link4  = tft.quaternion_matrix((
            tf_link4.transform.rotation.x,
            tf_link4.transform.rotation.y,
            tf_link4.transform.rotation.z,
            tf_link4.transform.rotation.w
        ))
        z4 = R_link4[:3, 2]    # link4 z-axis (plane normal)

        tf_link5  = self.tfb.lookup_transform('base_link', 'link5', rclpy.time.Time())
        link5_org = np.array([
            tf_link5.transform.translation.x,
            tf_link5.transform.translation.y,
            tf_link5.transform.translation.z
        ])
        vec      = st_base - link5_org
        # (orientation 계산에서 사용하므로 기존 정규화 벡터도 유지)
        vec_norm = vec / (np.linalg.norm(vec) + 1e-12)

        # link4 XY 평면으로 사영
        v_proj   = vec - np.dot(vec, z4) * z4
        n_proj   = np.linalg.norm(v_proj)
        if n_proj < 1e-6:
            # 사영이 거의 0이면 link4 x축을 안전한 대체 방향으로 사용
            x4   = R_link4[:3, 0]
            v_dir = x4 / (np.linalg.norm(x4) + 1e-12)
        else:
            v_dir = v_proj / n_proj

        # 최종 접근점: 사영된 평면 방향으로 offset, 그리고 link4 z축으로 0.04 상승
        appr_pos = st_base + 0.02 * z4 - offset_z * v_dir
        # # ④ EEF orientation: x-axis를 vec 방향으로 정렬
        # x_axis = vec_norm
        # z_ref  = np.array([0.0, 0.0, 1.0])
        # y_axis = np.cross(z_ref, x_axis)
        # if np.linalg.norm(y_axis) < 1e-4:
        #     y_axis = np.array([0.0, 1.0, 0.0])
        # y_axis /= np.linalg.norm(y_axis)
        # z_axis_eef = np.cross(x_axis, y_axis)
        # # R_eef = np.eye(4)
        # # R_eef[:3, 0] = x_axis
        # # R_eef[:3, 1] = y_axis
        # # R_eef[:3, 2] = z_axis_eef
        # # q_eef = tft.quaternion_from_matrix(R_eef)

        # # --- 축 재매핑 ---
        # # 요구사항:
        # #   현재 x축 → z축,  y축 → x축,  z축 → y축
        # # 즉, new_x = old_y, new_y = old_z, new_z = old_x
        # R_eef = np.eye(4)
        # # col0(x) := old y
        # R_eef[:3, 0] = y_axis
        # # col1(y) := old z
        # R_eef[:3, 1] = z_axis_eef
        # # col2(z) := old x
        # R_eef[:3, 2] = x_axis
        # q_eef = tft.quaternion_from_matrix(R_eef)

        # ④ EEF orientation (y축 고정):
        #    - link5의 y축은 유지
        #    - (딸기 - link5) 벡터를 link5의 XZ 평면에 사영
        #    - 사영된 벡터 방향으로 z축을 정렬
        #    - x축은 y × z로 직교화(우수좌법 유지)
        #
        #    참고: link5의 현재 축은 R_link5의 컬럼 벡터
        R_link5 = tft.quaternion_matrix((
            tf_link5.transform.rotation.x,
            tf_link5.transform.rotation.y,
            tf_link5.transform.rotation.z,
            tf_link5.transform.rotation.w
        ))
        x5 = R_link5[:3, 0]
        y5 = R_link5[:3, 1]   # 🔒 고정해야 하는 축
        z5 = R_link5[:3, 2]
        # (1) vec을 y5에 수직한 평면(= link5 XZ 평면)에 사영
        v_proj = vec_norm - np.dot(vec_norm, y5) * y5
        n_v = np.linalg.norm(v_proj)
        if n_v < 1e-6:
            # 사영이 거의 0이면 현재 z축 유지(특이상황)
            z_axis_eef = z5 / np.linalg.norm(z5)
        else:
            z_axis_eef = v_proj / n_v
        # (2) y축은 유지, x축은 y × z 로 결정(정규화)
        y_axis = y5 / np.linalg.norm(y5)
        x_axis = np.cross(y_axis, z_axis_eef)
        nx = np.linalg.norm(x_axis)
        if nx < 1e-6:
            # 혹시라도 수치 문제면 기존 x5 사용
            x_axis = x5 / np.linalg.norm(x5)
            # z 재계산으로 직교화 시도
            z_axis_eef = np.cross(x_axis, y_axis)
            z_axis_eef /= (np.linalg.norm(z_axis_eef) + 1e-12)
        else:
            x_axis /= nx
            # z를 다시 한번 직교화(수치 안정)
            z_axis_eef = np.cross(x_axis, y_axis)
            z_axis_eef /= (np.linalg.norm(z_axis_eef) + 1e-12)

        # --- (필요한 프레임 매핑이 있을 경우) 축 재매핑 유지 ---
        # 기존 요구사항:
        #   현재 x축 → z축,  y축 → x축,  z축 → y축
        # 즉, 최종 회전행렬의 컬럼 순서를 [y, z, x]로 배치
        R_eef = np.eye(4)
        # R_eef[:3, 0] = y_axis        # new x  <- old y
        # R_eef[:3, 1] = z_axis_eef    # new y  <- old z
        # R_eef[:3, 2] = x_axis        # new z  <- old x
        R_eef[:3, 0] = x_axis        # new x  <- old y
        R_eef[:3, 1] = y_axis   # new y  <- old z
        R_eef[:3, 2] = z_axis_eef        # new z  <- old x
        q_eef = tft.quaternion_from_matrix(R_eef)

        # ─── 최종 approach pose 로그 ──────────────────────────
        self.logger.info(f"[TF] approach(base_link) = {appr_pos}")

        appro_pose_base = PoseStamped()
        appro_pose_base.header.frame_id = 'base_link'
        appro_pose_base.header.stamp = self.get_clock().now().to_msg()

        appro_pose_base.pose.position = Point(
            x=float(appr_pos[0]),
            y=float(appr_pos[1]),
            z=float(appr_pos[2]),
         )
        appro_pose_base.pose.orientation = Quaternion(
            x=float(q_eef[0]), y=float(q_eef[1]),
            z=float(q_eef[2]), w=float(q_eef[3]))

        # RViz 마커 두 개 ---------------------------------------------------
        self.publish_marker(st_pose_cam, id=0, color=(1.0, 0.0, 0.0))  # red
        self.publish_marker(appro_pose_base, id=1, color=(0.0, 1.0, 0.0))  # green

        # ── 접근 목표를 TF로도 publish (base_link -> approach_target) ──
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id  = 'approach_target'   # RViz에서 이 프레임 선택해 확인
        t.transform.translation.x = appro_pose_base.pose.position.x
        t.transform.translation.y = appro_pose_base.pose.position.y
        t.transform.translation.z = appro_pose_base.pose.position.z
        t.transform.rotation      = appro_pose_base.pose.orientation
        self.tf_dyn.sendTransform(t)

        goal_handle.succeed()
        return DetectStrawberry.Result(approach_pose_base=appro_pose_base)

    # ----------------- RViz marker util ----------------------------
    def publish_marker(self, pose, id, color):
        m = Marker()
        m.header = pose.header
        m.ns = 'berry'
        m.id = id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = m.scale.y = m.scale.z = 0.03
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 0.8
        pub = self.marker_pub if id == 0 else self.appr_pub
        pub.publish(m)

    # ----------------- Gripper Action -----------------------------
    async def gripper_cb(self, goal_handle):
        self._grip_val = 1 if goal_handle.request.open else 0
        self.logger.info(f"[GRIPPER] → {'OPEN' if self._grip_val else 'CLOSE'}")
        goal_handle.succeed()
        return GripperControl.Result(done=True)

    def _tick_grip(self):
        msg = Int8(data=self._grip_val)
        self.grip_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(PerceptionNode())
    rclpy.shutdown()
