#!/usr/bin/env python3
# berry_perception/visualServoing.py
"""
IBVSBottomServo (업데이트)
 - 상단 RGBD + 하단 RGB 동시 사용
 - 목표:
    (A) 하단: bbox 중심 x == gripper ROI 중심 x  → 카메라 z축 회전(wz)만 유지
    (B) 상단 y2 키포인트와 하단 y1 키포인트가 같은 물리점이라 가정.
        **삼각측량은 RGBD(상단) 카메라의 ray 위**에서만 수행(깊이만 추정) → 타깃 3D 포인트 계산
    (C) gripper tip(link5 원점에서 z축 +0.13m)이 그 포인트와 일치하도록 선속도 제어
        └ 우선 z축 정렬(|ez|>tol_z 이면 vz만), 이후 xy 정렬(vx,vy)
 - 제어:
    - 선속도: vx, vy, vz (base_link frame)
    - 각속도: wz (bbox x 정렬), wx=0, wy=0  ← 각도 서보 삭제
 - 프레임 변환:
    bottom_camera_link / camera_link (body)  ←(고정)→ optical
    base_link ← cam_link(들)  … TF로 조회
 - ServoTwist 액션으로 servo 모드 진입 후 /eef_twist_cmd 에 Twist 퍼블리시
"""
import os, time, math
import numpy as np, cv2, torch, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

from berry_interface.action import ServoTwist, VisualServoing

# stem_and_yolo: 기존 로직 그대로 사용
from .stem_and_yolo import (
    select_yolo_by_roi_similarity, paint_polygons_with_hsv, get_bottom_mask_polygons,
    BOTTOM_MASK_ENABLE, BOTTOM_MASK_HSV, BOTTOM_MASK_FEATHER
)
# 헬퍼 모듈 (새로 추가)
from .ibvs_helpers import (
    R_LINK_FROM_OPT, xywh_to_xyxy, saturate, quat_to_R, boxes_conf_kpts,
    pixel_to_ray_opt, point_on_ray1_closest_to_ray2, draw_kpts,
    lookup_Rt_base_from_link, omega_base_from_omega_cam, publish_marker
)

class IBVSBottomServo(Node):
    def __init__(self):
        super().__init__(
            "ibvs_bottom_servo",
            automatically_declare_parameters_from_overrides=True
        )
        self.log = self.get_logger()
        self.bridge = CvBridge()

        # ───── 파라미터 ───────────────────────────────────────────
        # 콜백 그룹(재진입 허용): action, timer, subs 모두 같은 그룹으로 묶어 경합 방지
        self.cb_grp = ReentrantCallbackGroup()

        # 액션 서버 이름
        self.declare_parameter("visual_servo_action_name", "visual_servoing")
        # 액션 진행 중이 아닐 때는 제어 루프가 publish 하지 않도록 게이팅
        self._vs_active: bool = False
        self._vs_cancel_requested: bool = False
        self._vs_deadline: float = -1.0
 
        # 카메라 내부 파라미터 (기본값 = 네가 준 값)
        self.declare_parameter("fx", 636.235)
        self.declare_parameter("fy", 619.641)
        self.declare_parameter("cx", 211.89)
        self.declare_parameter("cy", 221.05)
 
        # 상단 RGBD 토픽
        self.declare_parameter("top_rgb_topic",   "/camera/camera/color/image_raw")
        self.declare_parameter("top_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("top_info_topic",  "/camera/camera/color/camera_info")
        self.declare_parameter("top_grip_roi_xywh", [498,250, 180, 228])

        # 프레임 이름
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("eef_frame",  "link5")
        self.declare_parameter("cam_frame",  "bottom_camera_link")   # bottom 카메라 body frame
        self.declare_parameter("top_cam_frame", "camera_link")       # top RGBD body frame
 
        # 하단 카메라 입력
        self.declare_parameter("use_v4l", True)
        self.declare_parameter("v4l_index", 0)
        self.declare_parameter("bottom_rgb_topic", "/bottom_cam/image_raw")

        # YOLO
        self.declare_parameter("yolo_model_rel", "model/model3.pt")
        self.declare_parameter("yolo_conf", 0.01)
        self.declare_parameter("yolo_sim_conf_thresh", 0.30)  # (IoU + conf) 점수 임계값

        # Gripper ROI (bbox x-align 기준)
        self.declare_parameter("grip_roi_xywh", [63, 83, 360, 471])

        # 제어 게인/제약
        # ▷ 제어/추론 주기 분리
        self.declare_parameter("ctrl_rate_hz", 10.0)     # 제어 루프 Hz (기본 3)
        self.declare_parameter("yolo_rate_hz", 3.0)     # YOLO 추론 Hz (기본 제어와 동일, 독립 조절)

        # 각속도 (wz) 게인
        self.declare_parameter("K_wz_center", 0.8)     # (u_err/fx)(rad) → wz_cam
        self.declare_parameter("w_limit", 0.5)         # 각속도 절대 최대 [rad/s]
        # 선속도 게인/제약
        self.declare_parameter("K_vx", 1.5)
        self.declare_parameter("K_vy", 1.5)
        self.declare_parameter("K_vz", 1.5)
        self.declare_parameter("v_limit", 0.2)        # [m/s]
        self.declare_parameter("tip_offset_z", 0.13)   # link5 z축 +0.13m
        self.declare_parameter("tol_z", 0.02)          # [m]  z 게이트 임계
        self.declare_parameter("tol_xy", 0.02)         # [m]  xy 종료 임계
 


        self.declare_parameter("ang_tol_deg", 2.0)     # [deg]   종료 기준
        self.declare_parameter("ux_tol_px",   4.0)     # [px]    종료 기준
        self.declare_parameter("steady_count_req", 8)  # 조건 만족 프레임 누적

        # Debug 창
        self.declare_parameter("show_debug", True)
        # 디버깅 옵션
        self.declare_parameter("debug_triangulation", False)
        self.declare_parameter("rviz_show_rays", True)

        # 타깃 오프셋(베이스 좌표계 +Z, m)
        self.declare_parameter("target_offset_z_base", 0.02)

        # ───── 상태 변수 ───────────────────────────────────────────
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)

        self.base_frame = self.get_parameter("base_frame").value
        self.eef_frame  = self.get_parameter("eef_frame").value
        self.cam_frame  = self.get_parameter("cam_frame").value
        self.top_cam_frame = self.get_parameter("top_cam_frame").value

        self.grip_roi = [int(v) for v in self.get_parameter("grip_roi_xywh").value]
        self.top_grip_roi = [int(v) for v in self.get_parameter("top_grip_roi_xywh").value]
 
        # 카메라 입력 (VideoCapture 또는 토픽)
        self.cap = None
        self.bot_img = None
        if bool(self.get_parameter("use_v4l").value):
            idx = int(self.get_parameter("v4l_index").value)
            self.cap = cv2.VideoCapture(idx)
            if not self.cap.isOpened():
                self.log.error(f"[camera] cannot open VideoCapture index {idx}")
                self.cap = None
        else:
            topic = self.get_parameter("bottom_rgb_topic").value
            self.sub_bot = self.create_subscription(Image, topic, self.cb_bottom_rgb, 10)
 
        # 상단 RGBD 구독
        self.top_rgb  = None
        self.top_depth = None
        self.top_info  = None
        self.create_subscription(Image, self.get_parameter("top_rgb_topic").value,   self._cb_top_rgb,   10)
        self.create_subscription(Image, self.get_parameter("top_depth_topic").value, self._cb_top_depth, 10)
        self.create_subscription(CameraInfo, self.get_parameter("top_info_topic").value, self._cb_top_info, 10)

        # YOLO 로드
        pkg_share  = get_package_share_directory('berry_perception')
        model_path = os.path.join(pkg_share, self.get_parameter("yolo_model_rel").value)
        self.log.info(f"[YOLO] loading: {model_path}")
        self.yolo = YOLO(model_path)
        self.yolo.model.to('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.yolo_conf = float(self.get_parameter("yolo_conf").value)
        self.sim_conf_thresh = float(self.get_parameter("yolo_sim_conf_thresh").value)
        # 클래스 이름 맵
        try:
            self._class_names = dict(self.yolo.model.names)
        except Exception:
            self._class_names = {}

        # TF
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        self._last_tf_warn_sec = 0.0

        # 퍼블리셔 (EEF twist 명령)
        self.twist_pub = self.create_publisher(Twist, "/eef_twist_cmd", 10)
        # RViz marker (타깃 3D 포인트)
        self.marker_pub = self.create_publisher(Marker, "ibvs_target_point", 10)

        # ServoTwist 액션 클라이언트 (내부 서보 모드 진입/해제)
        self.servo_client = ActionClient(self, ServoTwist, "servo_twist")
        self._servo_started: bool = False
        self._requested_stop: bool = False
        self._servo_goal_handle = None  # cancel용 핸들
        self._servo_pending: bool = False  # 중복 진입 방지

        # 하단 집게 마스킹 폴리곤
        self._bottom_polys = get_bottom_mask_polygons()

        # 종료 판단 변수
        self._steady_ok = 0
        self._reached_flag = False   # 액션 스레드와 공유

        # 지난 프레임에서 성공적으로 계산된 타깃 3D 포인트(베이스 좌표계)
        self._last_target_base = None
        # YOLO 추론 캐시 & 주기 분리용 상태
        self._last_yolo_time_s = 0.0
        self._yolo_seq = 0                    # 매 추론 시 증가
        self._last_accum_seq = -1             # 누적(클래스 합산)은 새 시퀀스마다 1회만
        self._last_results_bottom = None      # ultralytics result list (bottom)
        self._last_results_top = None         # ultralytics result list (top)
        self._last_bottom_xyxy = []
        self._last_bottom_conf = []
        self._last_bottom_cls  = []
        self._last_top_xyxy = []
        self._last_top_conf = []
        self._last_top_cls  = []
        # 액션 기간 동안 클래스별 conf 누적
        self._class_conf_sum = {}             # {class_name: summed_conf}

        # 디버그 주기 제한용 타임스탬프
        self._last_tri_dump_s   = 0.0
        self._last_tf_dump_s    = 0.0
        self._last_once_shape   = False

        # 디버그 이미지 퍼블리시 (RViz용)
        self.show_debug = bool(self.get_parameter("show_debug").value)

        # after (기본 QoS = Reliable 사용)
        self.pub_dbg_bottom = self.create_publisher(Image, "/ibvs/bottom/debug_image", 10)
        self.pub_dbg_top    = self.create_publisher(Image, "/ibvs/top/debug_image", 10)

        # 제어 루프
        hz = float(self.get_parameter("ctrl_rate_hz").value)
        self.ctrl_dt = 1.0 / max(1e-3, hz)
        self.timer = self.create_timer(self.ctrl_dt, self.tick, callback_group=self.cb_grp)
 

        # self.log.info("▶ IBVSBottomServo started. Will enter servo mode automatically.")
        # ── VisualServoing 액션 서버 개시 ─────────────────────────
        self.vs_server = ActionServer(
            self,
            VisualServoing,
            self.get_parameter("visual_servo_action_name").value,
            execute_callback=self.exec_vs,
            goal_callback=self.vs_goal_cb,
            cancel_callback=self.vs_cancel_cb,
            callback_group=self.cb_grp)

        self.log.info("▶ IBVSBottomServo started. Waiting for VisualServoing goals.")

        # (상수는 ibvs_helpers.R_LINK_FROM_OPT 사용)


    # ──────────────────────────────────────────────────────────────
    # Debug helpers
    # ──────────────────────────────────────────────────────────────
    def _dbg(self) -> bool:
        return bool(self.get_parameter("debug_triangulation").value)

    def _dinfo(self, msg: str):
        if self._dbg():
            self.log.info(msg)
    def _dwarn(self, msg: str):
        if self._dbg():
            self.log.warn(msg)

    def _every(self, sec: float, *, key: str = "tri") -> bool:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if key == "tri":
            if now_s - self._last_tri_dump_s > sec:
                self._last_tri_dump_s = now_s
                return True
            return False
        if key == "tf":
            if now_s - self._last_tf_dump_s > sec:
                self._last_tf_dump_s = now_s
                return True
            return False
        return True

    def _publish_ray(self, origin_base: np.ndarray, dir_base: np.ndarray,
                     color=(0.2, 0.9, 0.9, 0.9), ns="ray", mid=100, length=0.5):
        if not bool(self.get_parameter("rviz_show_rays").value):
            return
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns; m.id = mid; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.005  # line width
        m.color.r, m.color.g, m.color.b, m.color.a = color
        p0 = Point(x=float(origin_base[0]), y=float(origin_base[1]), z=float(origin_base[2]))
        end = origin_base + length * (dir_base / (np.linalg.norm(dir_base) + 1e-12))
        p1 = Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))
        m.points = [p0, p1]
        self.marker_pub.publish(m)

    # ──────────────────────────────────────────────────────────────
    # Callbacks & helpers
    # ──────────────────────────────────────────────────────────────
    def cb_bottom_rgb(self, msg: Image):
        self.bot_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # TOP RGBD callbacks
    def _cb_top_rgb(self, msg: Image):
        self.top_rgb = msg
    def _cb_top_depth(self, msg: Image):
        try:
            self.top_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32) / 1000.0
        except Exception:
            self.top_depth = None
    def _cb_top_info(self, msg: CameraInfo):
        self.top_info = msg

    def _start_servo_if_needed(self):
        """내부 ServoTwist 서버에 진입(한 번만). 액션 핸들을 잡아 나중에 cancel."""
        if self._servo_started or self._servo_pending:
            return
        if not self.servo_client.wait_for_server(timeout_sec=0.0):
            # 아직 서버 준비 안됐으면 다음 tick에 다시 시도
            return
        goal = ServoTwist.Goal()
        # future = self.servo_client.send_goal_async(goal)
        self._servo_pending = True
        future = self.servo_client.send_goal_async(goal)
        def _on_goal_done(fut):
            try:
                self._servo_goal_handle = fut.result()
                if self._servo_goal_handle is None:
                    self.log.warn("[servo] goal handle is None (failed to enter servo?)")
                else:
                    self.log.info("[servo] ENTER servo mode (ServoTwist goal accepted)")
            
                self._servo_started = (self._servo_goal_handle is not None)
                # ▷ 결과 콜백도 등록: 취소 없이 서버가 스스로 끝난 경우에도 정리
                if self._servo_goal_handle is not None:
                    res_fut = self._servo_goal_handle.get_result_async()
                    def _on_servo_result(_rf):
                        try:
                            _ = _rf.result()  # 결과 객체
                            self.log.info("[servo] result received (server finished)")
                        except Exception as e:
                            self.log.warn(f"[servo] get_result callback exception: {e}")
                        finally:
                            # 어떤 이유로든 끝났다면 다음 goal 위해 깔끔히 리셋
                            self._servo_goal_handle = None
                            self._servo_started = False
                            self._servo_pending = False
                    res_fut.add_done_callback(_on_servo_result)


            except Exception as e:
                self.log.warn(f"[servo] failed to get goal handle: {e}")
            finally:
                self._servo_pending = False
        future.add_done_callback(_on_goal_done)

    def _reset_vs_state(self):
        """다음 VisualServoing goal을 위해 내부 상태를 일괄 초기화."""
        self._vs_active = False
        self._vs_cancel_requested = False
        self._vs_deadline = -1.0
        self._reached_flag = False
        self._steady_ok = 0
        self._last_target_base = None
        # YOLO/클래스 누적 상태 초기화
        self._class_conf_sum = {}
        self._last_yolo_time_s = 0.0
        self._yolo_seq = 0
        self._last_accum_seq = -1
        self._last_results_bottom = None
        self._last_results_top = None
        self._last_bottom_xyxy = []
        self._last_bottom_conf = []
        self._last_bottom_cls  = []
        self._last_top_xyxy = []
        self._last_top_conf = []
        self._last_top_cls  = []

        # ServoTwist 관련
        self._requested_stop = False
        self._servo_pending = False
        self._servo_started = False
        self._servo_goal_handle = None

    def _stop_servo(self):
        """servo 종료 & 속도 0 보장"""
        if not self._requested_stop:
            self._requested_stop = True
            self.log.info("[servo] request STOP")
        # zero twist 1회 더 퍼블리시
        msg = Twist()
        self.twist_pub.publish(msg)
        # ServoTwist 취소 시도 (액션 핸들이 있으면 그걸, 없으면 전부 취소)
        try:
            # # if self._servo_goal_handle is not None:
            # #     self.servo_client.cancel_goal_async(self._servo_goal_handle)
            # if self._servo_goal_handle is not None:
            #     # goal handle에서 직접 취소
            #     self._servo_goal_handle.cancel_goal_async()
            # else:
            #     self.servo_client.cancel_all_goals()
            # self.log.info("[servo] CANCEL sent to ServoTwist server")
            if self._servo_goal_handle is not None:
                cancel_future = self._servo_goal_handle.cancel_goal_async()
                def _on_cancel_done(_fut):
                    try:
                        res = _fut.result()
                        self.log.info(f"[servo] CANCEL result: accepted={getattr(res, 'return_code', 'n/a')}")
                    except Exception as e:
                        self.log.warn(f"[servo] cancel callback exception: {e}")
                    finally:
                        # 핸들/플래그 정리
                        self._servo_goal_handle = None
                        self._servo_started = False
                        self._servo_pending = False
                        self._requested_stop = False
                cancel_future.add_done_callback(_on_cancel_done)
            else:
                self.log.info("[servo] no active ServoTwist goal handle to cancel")
        except Exception as e:
            self.log.warn(f"[servo] cancel to ServoTwist failed: {e}")
            # 예외가 나도 다음 goal은 받아야 하므로 강제 리셋
            self._servo_goal_handle = None
            self._servo_started = False
            self._servo_pending = False
            self._requested_stop = False

    # ──────────────────────────────────────────────────────────────
    # Main control loop
    # ──────────────────────────────────────────────────────────────
    def tick(self):
        
        # 액션이 활성화된 경우에만 동작
        if not self._vs_active:
            return
        # 0) 내부 ServoTwist 진입 시도
        self._start_servo_if_needed()

        # 1) 프레임 획득
        raw = None
        if self.cap is not None:
            ok, raw = self.cap.read()
            if not ok:
                raw = None
        else:
            raw = self.bot_img
        if raw is None:
            return

        frame = raw.copy()

        # 1.5) 입력 사이즈/내부파라미터 1회 로그
        if not self._last_once_shape and bool(self.get_parameter("debug_triangulation").value):
            self._last_once_shape = True
            bot_h, bot_w = frame.shape[:2]
            self.log.info(f"[dbg:init] bottom_img={bot_w}x{bot_h}  bot_K(fx,fy,cx,cy)=({self.fx:.2f},{self.fy:.2f},{self.cx:.2f},{self.cy:.2f})")

        # (선택) 그리퍼 마스킹 전처리
        if BOTTOM_MASK_ENABLE and len(self._bottom_polys) > 0:
            frame, _ = paint_polygons_with_hsv(
                frame,
                self._bottom_polys,
                hsv_color=BOTTOM_MASK_HSV,
                feather=int(BOTTOM_MASK_FEATHER),
                alpha=1.0
            )

        # 2) 하단 YOLO + ROI-유사도 선택
        #    ▷ YOLO는 yolo_rate_hz로만 수행, 제어는 ctrl_rate_hz로 캐시 사용

        gx, gy, gw, gh = self.grip_roi
        gr_xyxy = xywh_to_xyxy(gx, gy, gw, gh)

        # ── YOLO 추론 주기 제어 ──
        now_s = self.get_clock().now().nanoseconds * 1e-9
        yolo_dt = 1.0 / max(1e-3, float(self.get_parameter("yolo_rate_hz").value))
        if (now_s - self._last_yolo_time_s) >= yolo_dt:
            # bottom run
            results_b = self.yolo.predict(source=frame, conf=self.yolo_conf,
                                          save=False, verbose=False, max_det=10)
            self._last_results_bottom = results_b
            self._last_bottom_xyxy, self._last_bottom_conf, self._last_bottom_cls = [], [], []
            if results_b and results_b[0].boxes is not None and len(results_b[0].boxes) > 0:
                bb = results_b[0].boxes
                try:
                    arr_xy = bb.xyxy.detach().cpu().numpy()
                    arr_cf = bb.conf.detach().cpu().numpy()
                    arr_cl = bb.cls.detach().cpu().numpy()
                except Exception:
                    arr_xy = bb.xyxy.numpy(); arr_cf = bb.conf.numpy(); arr_cl = bb.cls.numpy()
                self._last_bottom_xyxy = arr_xy.astype(int).tolist()
                self._last_bottom_conf = [float(c) for c in arr_cf.tolist()]
                self._last_bottom_cls  = [int(c) for c in arr_cl.tolist()]
            # top run (이미지 있을 때만)
            if self.top_rgb is not None:
                try:
                    top_bgr_for_yolo = self.bridge.imgmsg_to_cv2(self.top_rgb, 'bgr8')
                    results_t = self.yolo.predict(source=top_bgr_for_yolo, conf=self.yolo_conf,
                                                  save=False, verbose=False, max_det=10)
                    self._last_results_top = results_t
                    self._last_top_xyxy, self._last_top_conf, self._last_top_cls = [], [], []
                    if results_t and results_t[0].boxes is not None and len(results_t[0].boxes) > 0:
                        tb = results_t[0].boxes
                        try:
                            arr_xy = tb.xyxy.detach().cpu().numpy()
                            arr_cf = tb.conf.detach().cpu().numpy()
                            arr_cl = tb.cls.detach().cpu().numpy()
                        except Exception:
                            arr_xy = tb.xyxy.numpy(); arr_cf = tb.conf.numpy(); arr_cl = tb.cls.numpy()
                        self._last_top_xyxy = arr_xy.astype(int).tolist()
                        self._last_top_conf = [float(c) for c in arr_cf.tolist()]
                        self._last_top_cls  = [int(c) for c in arr_cl.tolist()]
                except Exception:
                    pass
            # 추론 시각/시퀀스 갱신
            self._last_yolo_time_s = now_s
            self._yolo_seq += 1

        # 캐시로부터 선택/제어 입력 구성
        xyxy_b = list(self._last_bottom_xyxy)
        conf_b = list(self._last_bottom_conf)
 

        idx, sim_iou, seg_roi, seg, score = select_yolo_by_roi_similarity(
            bgr=frame,
            gr_xyxy=gr_xyxy,
            xyxy_list=xyxy_b,
            conf_list=conf_b,
            score_thresh=self.sim_conf_thresh
        )
        yolo_valid = (idx is not None)


        # 3) 제어오차 계산
        #   (a) bbox 중심 x vs ROI 중심 x → wz_cam (회전)
        #   (b) 상/하 키포인트로 타깃 3D 추정 → 선속도(v)
        wy_cam = 0.0
        wz_cam = 0.0
        ux_err  = None
        tri_ok = False
        target_base = None
        using_prev = False

        # ROI 중심 x
        cx_roi = gx + 0.5 * gw
        if yolo_valid:
            x1, y1, x2, y2 = xyxy_b[idx]
            cx_box = 0.5 * (x1 + x2)
            ux_err = float(cx_box - cx_roi)                 # [px]
            # (a) wz_cam: u 편차 → θ_z ≈ u/fx
            theta_z = ux_err / max(1e-6, self.fx)          # [rad]
            wz_cam  = - float(self.get_parameter("K_wz_center").value) * theta_z
        else:
            wz_cam = 0.0

        # 디버그 오버레이(하단): 유효/무효 공통 처리 (미정의 변수 접근 방지)
        if self.show_debug:
            vis = frame.copy()
            cv2.rectangle(vis, (gx, gy), (gx+gw, gy+gh), (255,255,255), 1)
            if yolo_valid:
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.line(vis, (int(cx_roi), 0), (int(cx_roi), vis.shape[0]-1), (255,255,0), 1)
                cv2.circle(vis, (int(cx_box), int(0.5*(y1+y2))), 4, (0,0,255), -1)
                # 하단 키포인트 오버레이 (가능 시)
                try:
                    bkobj = getattr(self._last_results_bottom[0], "keypoints", None) if self._last_results_bottom else None
                    if bkobj is not None and bkobj.xy is not None:
                        bxy_all = bkobj.xy[idx].detach().cpu().numpy() if hasattr(bkobj.xy, "detach") else bkobj.xy[idx]
                        b_kpts = [(float(p[0]), float(p[1])) for p in bxy_all]
                        # 전체는 노랑, 우리가 쓰는 y1=index 1은 청록
                        draw_kpts(vis, b_kpts, color=(0,255,255), idx_text=True)
                        if len(b_kpts) >= 2:
                            cv2.circle(vis, (int(b_kpts[1][0]), int(b_kpts[1][1])), 6, (0,200,200), 2)
                except Exception:
                    pass
                txt = f"u_err={ux_err:.1f}px  wz_cam={wz_cam:.3f}"
            else:
                txt = f"YOLO invalid (score<=thr={self.sim_conf_thresh:.2f})"
            cv2.putText(vis, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,255,200), 2)

            # ▶ RViz용 디버그 이미지 퍼블리시
            try:
                msg_bot = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                msg_bot.header.stamp = self.get_clock().now().to_msg()
                msg_bot.header.frame_id = self.cam_frame
                self.pub_dbg_bottom.publish(msg_bot)
            except Exception as e:
                self.log.warn(f"[debug] bottom debug publish failed: {e}")
 

        # 4) 카메라프레임 → base_link 로 각속도 변환
        w_cam = np.array([0.0, 0.0, wz_cam], dtype=float)  # 각도 서보 삭제: wx=wy=0
        try:
            w_base = omega_base_from_omega_cam(self.tfb, self.base_frame, self.cam_frame, w_cam)
        except Exception as e:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s - self._last_tf_warn_sec > 1.0:
                self.log.warn(f"[tf] waiting base<-{self.cam_frame}: {e}")
                self.log.warn("      Check frame names or publish static TF. You can set param 'base_frame'.")
                self._last_tf_warn_sec = now_s
            msg = Twist()  # zero
            self.twist_pub.publish(msg)
            return


        # 5) 한계 (회전)
        w_lim = float(self.get_parameter("w_limit").value)
        w_base[0] = 0.0
        w_base[1] = 0.0
        w_base[2] = saturate(w_base[2], w_lim)

        # 6) 상단 RGBD 처리 + 삼각측량 (TOP ray 위 점만 채택)
        top_dbg = None
        top_valid = (self.top_rgb is not None)
        if top_valid:
            top_bgr = self.bridge.imgmsg_to_cv2(self.top_rgb, 'bgr8')
            tgx, tgy, tgw, tgh = self.top_grip_roi
            tgr_xyxy = xywh_to_xyxy(tgx, tgy, tgw, tgh)
            # (캐시된 top 결과 사용)
            res_top = self._last_results_top
            xyxy_t = list(self._last_top_xyxy)
            conf_t = list(self._last_top_conf)

            t_idx, t_iou, t_seg_roi, t_seg, t_score = select_yolo_by_roi_similarity(
                bgr=top_bgr, gr_xyxy=tgr_xyxy, xyxy_list=xyxy_t, conf_list=conf_t, score_thresh=self.sim_conf_thresh)
            top_valid = (t_idx is not None)

            if bool(self.get_parameter("debug_triangulation").value) and self._every(1.0, key="tri"):
                th, tw = top_bgr.shape[:2]
                if self.top_info is not None:
                    fx_t = float(self.top_info.k[0]); fy_t = float(self.top_info.k[4])
                    cx_t = float(self.top_info.k[2]); cy_t = float(self.top_info.k[5])
                    self.log.info(f"[dbg:top] top_img={tw}x{th}  top_K=({fx_t:.2f},{fy_t:.2f},{cx_t:.2f},{cy_t:.2f})  "
                                  f"ROI={tgr_xyxy}  selected_idx={t_idx}")
                    if not (0 <= cx_t <= tw and 0 <= cy_t <= th):
                        self.log.warn(f"[dbg:top] principal point (cx,cy)=({cx_t:.1f},{cy_t:.1f}) is outside image size {tw}x{th}")
                else:
                    self.log.warn("[dbg:top] top_info is None (no intrinsics yet)")

            if self.show_debug:
                top_dbg = top_bgr.copy()
                cv2.rectangle(top_dbg, (tgx,tgy), (tgx+tgw,tgy+tgh), (255,255,255), 1)
                if top_valid:
                    x1t,y1t,x2t,y2t = xyxy_t[t_idx]
                    cv2.rectangle(top_dbg, (x1t,y1t), (x2t,y2t), (0,255,0), 2)
                    # 상단 키포인트(전체=노랑, 우리가 쓰는 y2=idx 2=핑크)
                    try:
                        tkobj = getattr(res_top[0], "keypoints", None) if res_top else None
                        if tkobj is not None and tkobj.xy is not None:
                            txy_all = tkobj.xy[t_idx].detach().cpu().numpy() if hasattr(tkobj.xy, "detach") else tkobj.xy[t_idx]
                            t_kpts = [(float(p[0]), float(p[1])) for p in txy_all]
                            draw_kpts(top_dbg, t_kpts, color=(0,255,255), idx_text=True)
                            if len(t_kpts) >= 3:
                                cv2.circle(top_dbg, (int(t_kpts[2][0]), int(t_kpts[2][1])), 6, (255,0,255), 2)
                    except Exception:
                        pass
                    
                # ▶ RViz용 상단 디버그 이미지 퍼블리시
                try:
                    msg_top = self.bridge.cv2_to_imgmsg(top_dbg, encoding='bgr8')
                    # 상단은 원본 헤더가 있을 수 있으나, 타이밍 일관성을 위해 now 사용
                    msg_top.header.stamp = self.get_clock().now().to_msg()
                    msg_top.header.frame_id = self.top_cam_frame
                    self.pub_dbg_top.publish(msg_top)
                except Exception as e:
                    self.log.warn(f"[debug] top debug publish failed: {e}")
 

        # 삼각측량 조건: 양쪽 모두 valid + 필요한 키포인트 존재 (top y2, bottom y1)
        if yolo_valid and top_valid:
            # bottom keypoint y1
            by1 = None; ty2 = None
            bkpts = getattr(self._last_results_bottom[0], "keypoints", None) if (self._last_results_bottom and yolo_valid) else None
            tkpts = getattr(res_top[0], "keypoints", None) if (res_top and top_valid) else None

            if bkpts is not None and bkpts.xy is not None:
                try: bxy = bkpts.xy[idx].detach().cpu().numpy()
                except Exception: bxy = bkpts.xy[idx]
                if bxy is not None and len(bxy) >= 2:
                    by1 = (float(bxy[1][0]), float(bxy[1][1]))
            if tkpts is not None and tkpts.xy is not None:
                try: txy = tkpts.xy[t_idx].detach().cpu().numpy()
                except Exception: txy = tkpts.xy[t_idx]
                if txy is not None and len(txy) >= 3:
                    ty2 = (float(txy[2][0]), float(txy[2][1]))

            if (by1 is not None) and (ty2 is not None) and (self.top_info is not None):
                # Rays in optical frames
                fx_b, fy_b, cx_b, cy_b = self.fx, self.fy, self.cx, self.cy
                rb_opt = pixel_to_ray_opt(by1[0], by1[1], fx_b, fy_b, cx_b, cy_b)
                fx_t = float(self.top_info.k[0]); fy_t = float(self.top_info.k[4])
                cx_t = float(self.top_info.k[2]); cy_t = float(self.top_info.k[5])
                rt_opt = pixel_to_ray_opt(ty2[0], ty2[1], fx_t, fy_t, cx_t, cy_t)

                # base ← cam_link (R,t) and link←opt mapping
                try:
                    R_bcb, t_bcb = lookup_Rt_base_from_link(self.tfb, self.base_frame, self.cam_frame)
                    R_bct, t_bct = lookup_Rt_base_from_link(self.tfb, self.base_frame, self.top_cam_frame)
                except Exception as e:
                    self.log.warn(f"[tf] base<-cams failed: {e}")
                    R_bcb = R_bct = None

                if R_bcb is not None and R_bct is not None:
                    if bool(self.get_parameter("debug_triangulation").value) and self._every(1.0, key="tf"):
                        # TF 요약 (RPY & t)
                        def rpy_from_R(R):
                            sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
                            roll = math.atan2(R[2,1], R[2,2])
                            pitch = math.atan2(-R[2,0], sy)
                            yaw = math.atan2(R[1,0], R[0,0])
                            return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
                        rpy_bot = rpy_from_R(R_bcb); rpy_top = rpy_from_R(R_bct)
                        self.log.info(f"[dbg:tf] base<-bot  t=({t_bcb[0]:+.3f},{t_bcb[1]:+.3f},{t_bcb[2]:+.3f})  rpy(deg)={rpy_bot}")
                        self.log.info(f"[dbg:tf] base<-top  t=({t_bct[0]:+.3f},{t_bct[1]:+.3f},{t_bct[2]:+.3f})  rpy(deg)={rpy_top}")

                    # optical → link(camera_link) 회전 보정
                    R_b_from_bopt = R_bcb @ R_LINK_FROM_OPT
                    R_b_from_topt = R_bct @ R_LINK_FROM_OPT
                    # 베이스 좌표계에서 두 레이 정의 (TOP을 ray1로, BOTTOM을 ray2로)
                    pb = t_bcb
                    pt = t_bct
                    db = (R_b_from_bopt @ rb_opt.reshape(3,1)).reshape(3)
                    dt = (R_b_from_topt @ rt_opt.reshape(3,1)).reshape(3)

                    if bool(self.get_parameter("debug_triangulation").value) and self._every(0.5, key="tri"):
                        # 레이/각도/거리 로그
                        dotv = float(np.dot(db/(np.linalg.norm(db)+1e-12), dt/(np.linalg.norm(dt)+1e-12)))
                        dotv = max(-1.0, min(1.0, dotv))
                        ang = math.degrees(math.acos(dotv))
                        base_cam_dist = float(np.linalg.norm(pt - pb))
                        self.log.info(f"[dbg:ray] by1={by1} ty2={ty2}  "
                                      f"rb_opt=({rb_opt[0]:+.3f},{rb_opt[1]:+.3f},{rb_opt[2]:+.3f})  "
                                      f"rt_opt=({rt_opt[0]:+.3f},{rt_opt[1]:+.3f},{rt_opt[2]:+.3f})")
                        self.log.info(f"[dbg:ray] angle(db,dt)={ang:.2f} deg  | base distance between cams={base_cam_dist*1000:.1f} mm")
                        if ang < 1.0:
                            self.log.warn("[dbg:ray] Rays nearly parallel (<1 deg) → triangulation unstable")
                    # RViz에 레이 표시
                    self._publish_ray(pb, db, color=(0.9,0.6,0.2,0.8), ns="bot_ray", mid=201, length=0.6)
                    self._publish_ray(pt, dt, color=(0.2,0.6,0.9,0.8), ns="top_ray", mid=202, length=0.6)

                    # Triangulate: **TOP ray 위 최근접점만 사용** (깊이만 추정) + 전방(s>=0) 강제
                    P_top_raw, gap, s_top = point_on_ray1_closest_to_ray2(pt, dt, pb, db)
                    if s_top < 0.0:
                        self.log.warn(f"[tri] s_top<0 (behind TOP): s={s_top:.3f}, gap={gap*1000:.1f}mm → clamp to 0")
                        s_top = 0.0
                        P_top = pt + s_top * dt
                    else:
                        P_top = P_top_raw
                    target_base = P_top

                    # 최신 성공 타깃으로 저장 (다음 프레임 실패 시 사용)
                    try: self._last_target_base = target_base.copy()
                    except Exception: self._last_target_base = target_base
 

                    # ---- 목표점 오프셋 적용 (base +Z) ----
                    target_offset = float(self.get_parameter("target_offset_z_base").value)
                    target_goal = target_base + np.array([0.0, 0.0, target_offset], dtype=np.float64)
 
                    tri_ok = True       
                    # 원래 삼각측량 점 (초록)
                    publish_marker(self.marker_pub, self.base_frame, self.get_clock().now().to_msg(),
                                   target_base, color=(0.1,0.8,0.2), ns="ibvs", mid=11)
                    # 오프셋 적용 목표점 (주황)
                    publish_marker(self.marker_pub, self.base_frame, self.get_clock().now().to_msg(),
                                   target_goal,  color=(1.0,0.5,0.1), ns="ibvs_goal", mid=13)

                    # 두 카메라 optical 기준 깊이 확인
                    R_topopt_from_base   = R_b_from_topt.T
                    R_botopt_from_base   = R_b_from_bopt.T
                    X_top_opt = R_topopt_from_base @ (target_base - pt)
                    X_bot_opt = R_botopt_from_base @ (target_base - pb)
                    depth_top = float(X_top_opt[2]); depth_bot = float(X_bot_opt[2])
                    
                    if bool(self.get_parameter("debug_triangulation").value) and self._every(0.5, key="tri"):
                        self.log.info(f"[dbg:depth] X_top_opt=({X_top_opt[0]:+.3f},{X_top_opt[1]:+.3f},{X_top_opt[2]:+.3f})  "
                                      f"X_bot_opt=({X_bot_opt[0]:+.3f},{X_bot_opt[1]:+.3f},{X_bot_opt[2]:+.3f})  s_top={s_top:.3f}")


                    # if depth_top <= 0.0:
                    #     self.log.warn(f"[tri] TOP optical depth <= 0 (={depth_top:.3f} m) → check K/TF/kpt match")
                    # if gap > 0.15:
                    #     self.log.warn(f"[tri] large ray gap: {gap*1000:.1f} mm (geometry inconsistency suspected)")
                    # self.log.info(f"[tri] target_base=({target_base[0]:+.3f},{target_base[1]:+.3f},{target_base[2]:+.3f}) m | "
                    #               f"top_depth={depth_top:+.3f} m, bot_depth={depth_bot:+.3f} m, gap={gap*1000:.1f} mm (TOP ray only)")
                    if depth_top <= 0.0:
                        self._dwarn(f"[tri] TOP optical depth <= 0 (={depth_top:.3f} m) → check K/TF/kpt match")
                    if gap > 0.15:
                        self._dwarn(f"[tri] large ray gap: {gap*1000:.1f} mm (geometry inconsistency suspected)")
                    self._dinfo(f"[tri] target_base=({target_base[0]:+.3f},{target_base[1]:+.3f},{target_base[2]:+.3f}) m | "
                                f"top_depth={depth_top:+.3f} m, bot_depth={depth_bot:+.3f} m, gap={gap*1000:.1f} mm (TOP ray only)")

                else:
                    if bool(self.get_parameter("debug_triangulation").value) and self._every(1.0, key="tri"):
                        self.log.warn("[dbg] Missing TF for one or both cameras; cannot triangulate")
            else:
                if bool(self.get_parameter("debug_triangulation").value) and self._every(1.0, key="tri"):
                    self.log.warn(f"[dbg] triangulation prerequisites not met: "
                                  f"yolo_valid={yolo_valid}, top_valid={top_valid}, "
                                  f"by1={'ok' if by1 is not None else 'None'}, ty2={'ok' if ty2 is not None else 'None'}, "
                                  f"top_info={'ok' if self.top_info is not None else 'None'}")
        # ── (추가) 클래스 누적: 새 YOLO 시퀀스에서 선택 bbox가 유효하면 1회만 반영 ──
        if yolo_valid and (self._last_accum_seq != self._yolo_seq):
            try:
                cls_id = self._last_bottom_cls[idx]
                conf_v = float(self._last_bottom_conf[idx])
                cls_name = self._class_names.get(cls_id, str(cls_id))
                self._class_conf_sum[cls_name] = self._class_conf_sum.get(cls_name, 0.0) + conf_v
                self._last_accum_seq = self._yolo_seq
            except Exception:
                pass

        # ── 삼각측량 실패 시: 이전에 성공했던 타깃을 그대로 사용(업데이트 없음) ──
        if (not tri_ok) and (self._last_target_base is not None):
            target_base = self._last_target_base
            tri_ok = True
            using_prev = True
            # 디버그 로깅 및 마커(회색)로 이전 타깃 재사용을 표시
            self._dinfo("[tri] current frame failed → use previous target (no update)")
            try:
                # 원래 타깃(회색)
                publish_marker(self.marker_pub, self.base_frame, self.get_clock().now().to_msg(),
                               target_base, color=(0.6,0.6,0.6), ns="ibvs_prev", mid=15)
                # 오프셋 적용 목표점(연회색)
                target_offset = float(self.get_parameter("target_offset_z_base").value)
                target_goal_prev = target_base + np.array([0.0, 0.0, target_offset], dtype=np.float64)
                publish_marker(self.marker_pub, self.base_frame, self.get_clock().now().to_msg(),
                               target_goal_prev, color=(0.8,0.8,0.8), ns="ibvs_prev_goal", mid=16)
            except Exception:
                pass

        # 7) Gripper tip(기준점) & 선속도 계산
        vx = vy = vz = 0.0
        reached_lin = False
        if tri_ok and (target_base is not None):
            try:
                R_beef, t_beef = lookup_Rt_base_from_link(self.tfb, self.base_frame, self.eef_frame)
                z_eef = (R_beef @ np.array([0,0,1], dtype=np.float64).reshape(3,1)).reshape(3)
                tip_offset = float(self.get_parameter("tip_offset_z").value)
                tip = t_beef + tip_offset * z_eef
                # err = target_base - tip
                # 오프셋 적용된 목표점 사용
                target_offset = float(self.get_parameter("target_offset_z_base").value)
                target_goal = target_base + np.array([0.0, 0.0, target_offset], dtype=np.float64)
                # 이전 타깃 사용 중임을 마커 색으로도 한 번 더 표기(선택)
                if using_prev:
                    publish_marker(self.marker_pub, self.base_frame, self.get_clock().now().to_msg(),
                                   target_goal, color=(0.8,0.8,0.8), ns="ibvs_prev_goal", mid=16)

                err = target_goal - tip

                ez = float(err[2]); ex = float(err[0]); ey = float(err[1])
                tol_z = float(self.get_parameter("tol_z").value)
                tol_xy = float(self.get_parameter("tol_xy").value)
                # 게인
                Kvx = float(self.get_parameter("K_vx").value)
                Kvy = float(self.get_parameter("K_vy").value)
                Kvz = float(self.get_parameter("K_vz").value)
                v_lim = float(self.get_parameter("v_limit").value)
                # 게이팅: z 먼저
                if abs(ez) > tol_z:
                    vx = 0.0; vy = 0.0; vz = saturate(Kvz * ez, v_lim)
                else:
                    vz = saturate(Kvz * ez, v_lim)   # 미세 보정은 유지
                    vx = saturate(Kvx * ex, v_lim)
                    vy = saturate(Kvy * ey, v_lim)
                # 종료 판단
                if (abs(ez) <= tol_z) and (math.hypot(ex, ey) <= tol_xy):
                    self._steady_ok += 1
                else:
                    self._steady_ok = 0
                reached_lin = self._steady_ok >= int(self.get_parameter("steady_count_req").value)
                # 로깅
                self._dinfo(f"[lin] tip=({tip[0]:+.3f},{tip[1]:+.3f},{tip[2]:+.3f})  "
                            f"err=({ex:+.3f},{ey:+.3f},{ez:+.3f}) m  "
                            f"v_cmd=({vx:+.3f},{vy:+.3f},{vz:+.3f}) m/s")
                publish_marker(self.marker_pub, self.base_frame, self.get_clock().now().to_msg(),
                               tip, color=(0.2,0.4,1.0), ns="tip", mid=12)
            except Exception as e:
                self.log.warn(f"[tf] base<-{self.eef_frame} failed: {e}")
                self._steady_ok = 0


        # 7) Twist 퍼블리시
        msg = Twist()
        # 선속도 (base frame)
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.linear.z  = float(vz)
        # 각속도 (wz만)
        msg.angular.x = float(w_base[0])
        msg.angular.y = float(w_base[1])
        msg.angular.z = float(w_base[2])

        self.twist_pub.publish(msg)
        # self.log.info(f"[ibvs] bot_idx={idx if yolo_valid else 'None'} score={score:.2f} iou={sim_iou:.2f}  "
        #               f"u_err={ux_err if ux_err is not None else float('nan'):.1f}px  "
        #               f"w_cam=[0,0,{wz_cam:.3f}]→w_base=[{w_base[0]:.3f},{w_base[1]:.3f},{w_base[2]:.3f}]  "
        #               f"v=[{vx:.3f},{vy:.3f},{vz:.3f}]")
        self._dinfo(f"[ibvs] bot_idx={idx if yolo_valid else 'None'} score={score:.2f} iou={sim_iou:.2f}  "
                    f"u_err={ux_err if ux_err is not None else float('nan'):.1f}px  "
                    f"w_cam=[0,0,{wz_cam:.3f}]→w_base=[{w_base[0]:.3f},{w_base[1]:.3f},{w_base[2]:.3f}]  "
                    f"v=[{vx:.3f},{vy:.3f},{vz:.3f}]")
        

        # 8) 종료 처리 (선속도 조건 충족 시)
        if reached_lin and not self._requested_stop:
            self.log.info("[IBVS] ✔ target reached (pos tolerance met)")
            self._reached_flag = True
            # 여기서는 즉시 액션을 끝내지 않고, exec_vs 루틴이 정리/리턴을 담당하도록 플래그만 켭니다.
            # 안전상 서보/트위스트는 즉시 멈춤
            self._stop_servo()
            self._requested_stop = False


        # 키보드 종료
        if self.show_debug:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.log.info("⏹ Cancel by user (q) → request action cancel")
                self._vs_cancel_requested = True
                # 즉시 ServoTwist 정지
                self._stop_servo()


    # ──────────────────────────────────────────────────────────────
    # VisualServoing Action Server 콜백들
    # ──────────────────────────────────────────────────────────────
    def vs_goal_cb(self, goal_request: VisualServoing.Goal):
        if self._vs_active:
            self.log.warn("[vs_goal_cb] already running → REJECT new goal")
            return GoalResponse.REJECT
        if goal_request.timeout_sec <= 0.0:
            self.log.warn("[vs_goal_cb] invalid timeout (<=0) → REJECT")
            return GoalResponse.REJECT
        self.log.info(f"[vs_goal_cb] ACCEPT goal (timeout={goal_request.timeout_sec:.2f}s)")
        return GoalResponse.ACCEPT

    def vs_cancel_cb(self, cancel_request):
        self.log.info("[vs_cancel_cb] CANCEL requested → ACCEPT")
        self._vs_cancel_requested = True
        # 즉시 내부 서보 정지 시그널
        self._stop_servo()
        return CancelResponse.ACCEPT

    def exec_vs(self, goal_handle):
        req = goal_handle.request
        self.log.info(f"[exec_vs] ▶ START VisualServoing (timeout={req.timeout_sec:.2f}s)")
        # 실행 상태 초기화(다음 goal에 영향 없도록 clean start)
        self._reset_vs_state()
        self._vs_active = True

        t0 = self.get_clock().now().nanoseconds * 1e-9
        self._vs_deadline = t0 + float(req.timeout_sec)

        try:
            # 메인 대기 루프: tick()이 플래그를 셋업하면 종료
            while rclpy.ok() and self._vs_active:
                now_s = self.get_clock().now().nanoseconds * 1e-9
                # 피드백
                fb = VisualServoing.Feedback()
                fb.time_elapsed = float(now_s - t0)
                fb.state = "running"
                goal_handle.publish_feedback(fb)
 
                # 취소 요청
                if goal_handle.is_cancel_requested or self._vs_cancel_requested:
                    self.log.info("[exec_vs] ✖ CANCELED by client/user")
                    self._stop_servo()
                    self._vs_active = False
                    goal_handle.canceled()
                    return VisualServoing.Result(success=False, message="Canceled")

                # 목표 도달
                if self._reached_flag:
                    self.log.info("[exec_vs] ✔ REACHED → stopping")
                    self._vs_active = False
                    # 이미 _stop_servo()는 tick 쪽에서 호출됨(안전차원으로 한 번 더 0 퍼블리시)
                    self._stop_servo()
                    goal_handle.succeed()
                    # 결과: 누적 클래스 집계
                    cls_name, cls_sum = self._decide_class()
                    return VisualServoing.Result(success=True, message="Reached",
                                                 fruit_class=str(cls_name), fruit_class_conf_sum=float(cls_sum))

                    
                # 타임아웃
                if now_s > self._vs_deadline:
                    self.log.warn("[exec_vs] ⏰ TIMEOUT → stopping")
                    self._stop_servo()
                    self._vs_active = False
                    goal_handle.abort()
                    cls_name, cls_sum = self._decide_class()
                    return VisualServoing.Result(success=False, message="Timeout",
                                                 fruit_class=str(cls_name), fruit_class_conf_sum=float(cls_sum))

                    
                # 50ms 간격으로 상태 갱신
                time.sleep(0.05)
        finally:
            # 어떤 경로로든 exec 종료 시 상태 정리
            try:
                self._stop_servo()
            finally:
                self._reset_vs_state()

        # 여기까지 오면 루프가 외부 종료
        if not rclpy.ok():
            self.log.warn("[exec_vs] node is shutting down")
        cls_name, cls_sum = self._decide_class()
        if self._reached_flag:
            goal_handle.succeed()
            return VisualServoing.Result(success=True, message="Reached",
                                         fruit_class=str(cls_name), fruit_class_conf_sum=float(cls_sum))
        elif self._vs_cancel_requested:
            goal_handle.canceled()
            return VisualServoing.Result(success=False, message="Canceled",
                                         fruit_class=str(cls_name), fruit_class_conf_sum=float(cls_sum))
        else:
            goal_handle.abort()
            return VisualServoing.Result(success=False, message="Aborted",
                                         fruit_class=str(cls_name), fruit_class_conf_sum=float(cls_sum))

    # ──────────────────────────────────────────────────────────────
    # Class decision helper
    # ──────────────────────────────────────────────────────────────
    def _decide_class(self):
        """액션 기간 동안 누적된 클래스별 conf 합에서 최대값을 반환"""
        if not self._class_conf_sum:
            return ("", 0.0)
        name = max(self._class_conf_sum, key=lambda k: self._class_conf_sum[k])
        return (name, float(self._class_conf_sum[name]))



def main():
    import rclpy.executors as execs
    rclpy.init()
    node = IBVSBottomServo()
    # 멀티스레드 실행기: Action execute loop와 timer(tick) 동시 실행 보장
    executor = execs.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
