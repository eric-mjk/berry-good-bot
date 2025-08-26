#!/usr/bin/env python3
import rclpy, math, cv2, torch, numpy as np, os, sys, time
from rclpy.node            import Node
from rclpy.action          import ActionServer, GoalResponse, CancelResponse
from sensor_msgs.msg       import Image, CameraInfo
from geometry_msgs.msg     import TransformStamped, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from tf2_ros               import StaticTransformBroadcaster, Buffer, TransformListener, TransformBroadcaster
import tf_transformations   as tft
from cv_bridge             import CvBridge
from ultralytics           import YOLO
from berry_interface.action import DetectStrawberry, GripperControl, VisualServoDetect
from berry_interface.msg    import DetectionBBox
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
        self.declare_parameter('rgb_camera_frame', 'rgb_cam')   # (사용 안 해도 무방)
        self.declare_parameter('rgb_cam_index', 0)              # (사용 안 해도 무방)

        # 상단 카메라(기존)
        # self.declare_parameter('camera_to_eef', [0.0, 0.08, 0.045,  math.pi/180*30, -math.pi/2, -math.pi/2]) # xyz + rpy
        self.declare_parameter('camera_to_eef', [0.035, 0.09, 0.045,  math.pi/180*0, -math.pi/2, -math.pi/2]) # xyz + rpy
        self.declare_parameter('camera_tilt_deg', 35.0)  # 상단 카메라 보정 틸트

        # 하단(아래) 카메라 TF 추가
        self.declare_parameter('bottom_camera_frame', 'bottom_camera_link')
        # 기본값은 예시이며 실제 로봇에 맞게 launch/YAML로 오버라이드하세요.
        self.declare_parameter('bottom_camera_to_eef', [0.0, -0.09, 0.0365, 0.0, -math.pi/2, -math.pi/2]) # xyz + rpy
        self.declare_parameter('bottom_camera_tilt_deg', -35.0)  # 필요 시 틸트 보정
        # 1단계 선택 시 conf 임계값(이상) 중 EEF와 최근접 후보 선택
        self.declare_parameter('stage1_min_conf', 0.4)

        cam2eef = self.get_parameter('camera_to_eef').value
        self.logger.debug(f"[PARAM] camera_to_eef = {cam2eef}")

        self.publish_static_tf(cam2eef, tilt_deg=float(self.get_parameter('camera_tilt_deg').value))
        bot_cam2eef = self.get_parameter('bottom_camera_to_eef').value
        self.logger.debug(f"[PARAM] bottom_camera_to_eef = {bot_cam2eef}")
        self.publish_bottom_tf(bot_cam2eef, tilt_deg=float(self.get_parameter('bottom_camera_tilt_deg').value))


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
        # ── Visual Servo 결과/디버그 토픽 ─────────────────────
        # (VisualServoDetect 관련 퍼블리셔 제거)

        # ── 간단 트래킹(프레임 간 동일 딸기 유지) ─────────────────
        self._prev_bbox_rgb  = None   # [x1,y1,x2,y2]
        self._prev_bbox_rgbd = None

        # ────── YOLOv8 초기화 ───────────────────────────────────────
        pkg_share  = get_package_share_directory('berry_perception')
        model_path = os.path.join(pkg_share, 'model', 'model3.pt')
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
    def publish_static_tf(self, xyzrpy, tilt_deg: float = 35.0):
        """상단 카메라 정적 TF publish (parent=eef_frame, child=camera_frame)."""
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

        # ── (2) 보정: 초록(Y)축 기준 tilt_deg 만큼 앞쪽으로 기울이기 ──
        tilt_rad = math.radians(float(tilt_deg))
        corr_mat = tft.rotation_matrix(tilt_rad, (0, 1, 0))

        # ── (3) 두 행렬 곱 → 최종 쿼터니언 ────────────────────
        final_q  = tft.quaternion_from_matrix(
                      tft.concatenate_matrices(base_rot, corr_mat))
        t.transform.rotation = Quaternion(
           x=final_q[0], y=final_q[1], z=final_q[2], w=final_q[3])
        br.sendTransform(t)

    def publish_bottom_tf(self, xyzrpy, tilt_deg: float = 0.0):
        """하단(아래) 카메라 정적 TF publish (parent=eef_frame, child=bottom_camera_frame)."""
        br = StaticTransformBroadcaster(self)
        t  = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('eef_frame').value
        t.child_frame_id  = self.get_parameter('bottom_camera_frame').value
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyzrpy[:3]
        # RPY → 행렬
        base_rot = tft.euler_matrix(*xyzrpy[3:])
        # 보정 틸트 (필요 시)
        tilt_rad = math.radians(float(tilt_deg))
        corr_mat = tft.rotation_matrix(tilt_rad, (0, 1, 0))
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
        # offset_z = goal_handle.request.offset_z
        # self.logger.info(f"[ACTION] new goal: offset_z={offset_z:.3f}")
        offset_z = goal_handle.request.offset_z
        stage    = getattr(goal_handle.request, "approach_stage", 1)
        prev_pt  = getattr(goal_handle.request, "prev_target_base", Point())
        prev_gate = getattr(goal_handle.request, "prev_gate_m", 0.0)
        self.logger.info(f"[ACTION] new goal: offset_z={offset_z:.3f}, stage={int(stage)}, gate={prev_gate:.3f}")

        # 최신 frame 올 때까지 대기
        while self.rgb_img is None or self.depth_img is None or self.cam_info is None:
            self.logger.info(f"[ACTION] no img!!")
            rclpy.spin_once(self, timeout_sec=0.1)

        cv_img = self.bridge.imgmsg_to_cv2(self.rgb_img, 'bgr8')
        results = self.model.predict(source=cv_img, conf=0.3, save=False, verbose=False)
        if not results or not results[0].boxes:
            self.get_logger().warn('No strawberry detected')
            goal_handle.abort()
            return DetectStrawberry.Result()

        # # 가장 높은 confidence 선택
        # boxes = results[0].boxes
        # best  = boxes[boxes.conf.argmax()]
        # self.logger.info(f"[YOLO] {len(boxes)} boxes, best conf={float(best.conf):.3f}")

        # x1, y1, x2, y2 = map(int, best.xyxy[0])
        # cx, cy = int((x1+x2)/2), int((y1+y2)/2)

        # # z값(dept) 확보
        # # depth = self.bridge.imgmsg_to_cv2(self.depth_img, 'passthrough')[cy, cx] / 1000.0  # mm→m
        # # if depth == 0.0:
        # #     goal_handle.abort()
        # #     return DetectStrawberry.Result()

        # 공통 준비물: depth / 카메라내부 / TF(cam->base)
        depth_np = self.bridge.imgmsg_to_cv2(self.depth_img, 'passthrough').astype(np.float32) / 1000.0
        fx = self.cam_info.k[0];  fy = self.cam_info.k[4]
        cx0 = self.cam_info.k[2]; cy0 = self.cam_info.k[5]
        try:
            tf_cam2base = self.tfb.lookup_transform('base_link', self.get_parameter('camera_frame').value, rclpy.time.Time())
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

        # 후보 생성 유틸: bbox → (cx,cy,depth)->cam→base
        def _bbox_center3d_base(x1,y1,x2,y2):
            # 강건 depth (bbox 하위 25% 평균)
            win = depth_np[max(0,y1):max(0,y2), max(0,x1):max(0,x2)].reshape(-1)
            vals = win[(win > 0) & np.isfinite(win)]
            if vals.size == 0:
                return None
            med  = np.median(vals); mad = np.median(np.abs(vals - med))
            thr  = 3 * 1.4826 * mad if mad > 0 else np.inf
            inl  = vals[np.abs(vals - med) < thr]
            srt  = np.sort(inl); qn = max(1, int(len(srt) * 0.25))
            d    = float(np.mean(srt[:qn]))
            if not np.isfinite(d) or d <= 0: return None
            cxp, cyp = int((x1+x2)/2), int((y1+y2)/2)
            x_opt = (cxp - cx0) * d / fx
            y_opt = (cyp - cy0) * d / fy
            z_opt = d
            X =  z_opt;  Y = -x_opt;  Z = -y_opt
            p_cam  = np.array([X, Y, Z, 1.0])
            p_base = (mat_cam2base @ p_cam)[:3]
            return (cxp, cyp, d, p_base)

        # ── Stage 분기 ──────────────────────────────────────────────
        boxes = results[0].boxes
        if int(stage) == 2:
            # 2단계: BT가 준 prev_target_base에 가장 가까운 후보 선택(게이트 적용 가능)
            px, py, pz = float(prev_pt.x), float(prev_pt.y), float(prev_pt.z)
            anchor = np.array([px, py, pz], dtype=float)
            best = None
            min_d = float('inf')
            for i in range(len(boxes)):
                x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
                got = _bbox_center3d_base(x1,y1,x2,y2)
                if got is None: 
                    continue
                cx, cy, d, p_base = got
                dd = float(np.linalg.norm(p_base - anchor))
                if prev_gate > 0.0 and dd > prev_gate:
                    continue
                if dd < min_d:
                    min_d = dd
                    best = (x1,y1,x2,y2,cx,cy,d,p_base)
            if best is None:
                self.get_logger().warn("Stage2: no candidate within gate or depth invalid")
                goal_handle.abort()
                return DetectStrawberry.Result()
            x1,y1,x2,y2,cx,cy,depth,st_base = best
            self.logger.info(f"[STAGE2] picked near anchor, dist={min_d:.3f} m")
        else:
        #     # 1단계: 기존 로직(최고 conf) 유지
        #     best_box = boxes[boxes.conf.argmax()]
        #     self.logger.info(f"[YOLO] {len(boxes)} boxes, best conf={float(best_box.conf):.3f}")
        #     x1, y1, x2, y2 = map(int, best_box.xyxy[0])
        #     cx, cy = int((x1+x2)/2), int((y1+y2)/2)
        #     got = _bbox_center3d_base(x1,y1,x2,y2)
        #     if got is None:
        #         self.logger.warn("No valid depth for best box → abort")
        #         goal_handle.abort()
        #         return DetectStrawberry.Result()
        #     cx, cy, depth, st_base = got
        #     self.logger.info(f"[STAGE1] depth={depth:.3f} m")

        # self.logger.info(f"[DEPTH] depth({cx},{cy}) = {depth:.3f} m")

            # 1단계: conf ≥ threshold 후보들 중 EEF(link5)와 가장 가까운 물체 선택
            min_conf = float(self.get_parameter('stage1_min_conf').value)
            try:
                tf_link5  = self.tfb.lookup_transform('base_link', 'link5', rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(f"TF link5 error: {e}")
                goal_handle.abort()
                return DetectStrawberry.Result()
            link5_org = np.array([
                tf_link5.transform.translation.x,
                tf_link5.transform.translation.y,
                tf_link5.transform.translation.z
            ], dtype=float)

            # 1차 패스: conf ≥ min_conf
            best = None
            min_d = float('inf')
            for i in range(len(boxes)):
                # confidence check
                try:
                    conf_i = float(boxes.conf[i].item())
                except Exception:
                    conf_i = float(boxes.conf[i].cpu().numpy())
                if conf_i < min_conf:
                    continue
                x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
                got = _bbox_center3d_base(x1,y1,x2,y2)
                if got is None:
                    continue
                cx_i, cy_i, d_i, p_base = got
                dd = float(np.linalg.norm(p_base - link5_org))
                if dd < min_d:
                    min_d = dd
                    best = (x1,y1,x2,y2,cx_i,cy_i,d_i,p_base)

            # 2차 패스: (fallback) threshold 만족 후보가 없으면 모든 후보 중 최근접
            if best is None:
                for i in range(len(boxes)):
                    x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
                    got = _bbox_center3d_base(x1,y1,x2,y2)
                    if got is None:
                        continue
                    cx_i, cy_i, d_i, p_base = got
                    dd = float(np.linalg.norm(p_base - link5_org))
                    if dd < min_d:
                        min_d = dd
                        best = (x1,y1,x2,y2,cx_i,cy_i,d_i,p_base)

            if best is None:
                self.get_logger().warn("Stage1: no candidate with valid depth")
                goal_handle.abort()
                return DetectStrawberry.Result()

            x1,y1,x2,y2,cx,cy,depth,st_base = best
            self.logger.info(f"[STAGE1] picked nearest to EEF, dist={min_d:.3f} m (min_conf={min_conf:.2f})")


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
        
        st_base = (mat_cam2base @ p_cam)[:3] if int(stage)!=2 else st_base  # stage2는 위에서 이미 산출

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
        return DetectStrawberry.Result(
            approach_pose_base=appro_pose_base,
            strawberry_center_base=Point(x=float(st_base[0]), y=float(st_base[1]), z=float(st_base[2]))
        )

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


    # ----------------- YOLO 결과 유틸 ------------------------------
    @staticmethod
    def _iou(a, b):
        # a,b: [x1,y1,x2,y2]
        ax1, ay1, ax2, ay2 = a; bx1, by1, bx2, by2 = b
        inter_x1 = max(ax1, bx1); inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2); inter_y2 = min(ay2, by2)
        iw = max(0, inter_x2 - inter_x1); ih = max(0, inter_y2 - inter_y1)
        inter = iw * ih
        a_area = max(0, ax2 - ax1) * max(0, ay2 - ay1)
        b_area = max(0, bx2 - bx1) * max(0, by2 - by1)
        union = a_area + b_area - inter + 1e-9
        return inter / union

    @staticmethod
    def _center_distance(a, b):
        ax = 0.5 * (a[0] + a[2]); ay = 0.5 * (a[1] + a[3])
        bx = 0.5 * (b[0] + b[2]); by = 0.5 * (b[1] + b[3])
        dx = ax - bx; dy = ay - by
        return (dx*dx + dy*dy) ** 0.5

    def _get_all_boxes(self, results):
        """returns (xyxy: Nx4 int list, conf: list[float]) or ([],[])"""
        if (not results) or (results[0].boxes is None) or (len(results[0].boxes) == 0):
            return [], []
        b = results[0].boxes
        try:
            xyxy = b.xyxy.cpu().numpy()
            conf = b.conf.cpu().numpy()
        except Exception:
            xyxy = b.xyxy.numpy() if hasattr(b.xyxy, "numpy") else np.array(b.xyxy)
            conf = b.conf.numpy() if hasattr(b.conf, "numpy") else np.array(b.conf)
        xyxy = xyxy.astype(int).tolist()
        conf = [float(c) for c in conf.tolist()]
        return xyxy, conf

    def _choose_track(self, prev_bbox, xyxy_list, conf_list):
        """prev가 있으면 IoU 최대(>0.1 우선), 아니면 conf 최대. IoU 낮으면 센터거리 최소."""
        if not xyxy_list:
            return None
        if prev_bbox is None:
            return int(np.argmax(conf_list))
        # IoU 기준
        ious = [self._iou(prev_bbox, bb) for bb in xyxy_list]
        best_iou = max(ious); i_best = int(np.argmax(ious))
        if best_iou > 0.1:
            return i_best
        # 센터 거리
        dists = [self._center_distance(prev_bbox, bb) for bb in xyxy_list]
        return int(np.argmin(dists))

    def _best_box(self, results):
        # if not results or not results[0].boxes:
        #     return None, None
        # boxes = results[0].boxes
        # idx = int(boxes.conf.argmax())
        # box = boxes[idx]
        # conf = float(boxes.conf[idx].item())
        # x1, y1, x2, y2 = map(int, box.xyxy[0])
        # return (x1, y1, x2, y2), conf

        if (not results) or (results[0].boxes is None) or (len(results[0].boxes) == 0):
            return None, None, None
        boxes = results[0].boxes
        # best index by confidence
        try:
            idx = int(torch.argmax(boxes.conf).item())
        except Exception:
            idx = int(np.argmax(boxes.conf.cpu().numpy()))
        x1, y1, x2, y2 = map(int, results[0].boxes.xyxy[idx].tolist())
        conf = float(results[0].boxes.conf[idx].item())
        return (x1, y1, x2, y2), conf, idx

    def _extract_kpts(self, results, idx):
        """
        returns: kpt_x(list[float]), kpt_y(list[float]), kpt_conf(list[float] or [])
        """
        try:
            rk = results[0].keypoints
        except Exception:
            rk = None
        if rk is None or rk.xy is None:
            return [], [], []
        # shape: (N, K, 2)
        try:
            xy = rk.xy[idx].cpu().numpy()
        except Exception:
            xy = rk.xy[idx]
        kpx = [float(p[0]) for p in xy]
        kpy = [float(p[1]) for p in xy]
        kpc = []
        if hasattr(rk, "conf") and rk.conf is not None:
            try:
                confv = rk.conf[idx].cpu().numpy().tolist()
            except Exception:
                confv = rk.conf[idx].tolist()
            kpc = [float(c) for c in confv]
        return kpx, kpy, kpc

    @staticmethod
    def _robust_depth_at(u_px: float, v_px: float, depth_np: np.ndarray) -> float:
        """주변 3x3 윈도우에서 robust 하게 깊이(m) 추정. 없으면 NaN."""
        if depth_np is None:
            return float('nan')
        h, w = depth_np.shape[:2]
        u = int(round(u_px)); v = int(round(v_px))
        u = max(0, min(w-1, u)); v = max(0, min(h-1, v))
        x0 = max(0, u-1); x1 = min(w, u+2)
        y0 = max(0, v-1); y1 = min(h, v+2)
        win = depth_np[y0:y1, x0:x1].reshape(-1)
        vals = win[(win > 0) & np.isfinite(win)]
        if vals.size == 0:
            return float('nan')
        sorted_vals = np.sort(vals)
        q25_len = max(1, int(len(sorted_vals) * 0.25))
        return float(np.mean(sorted_vals[:q25_len]))

def main():
    # rclpy.init()
    # rclpy.spin(PerceptionNode())
    # rclpy.shutdown()
    import rclpy.executors as execs
    rclpy.init()
    node = PerceptionNode()
    executor = execs.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

