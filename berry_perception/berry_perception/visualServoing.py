#!/usr/bin/env python3
"""
IBVSBottomServo
 - 하단 RGB 카메라 1개로 IBVS 수행
 - 목표:
    (1) y0–y1 키포인트 직선이 영상 y축과 평행(= 각도 0)
    (2) bbox 중심의 x좌표가 gripper ROI 중심 x좌표와 일치
 - 제어:
    - 선속도: vx=vy=vz=0
    - 각속도: wx=0, wy≠0, wz≠0  (optical frame에서 설계)
 - 프레임 변환:
    optical → camera_body(bottom_camera_link) → eef(link5) → base_link
 - ServoTwist 액션으로 servo 모드 진입 후 /eef_twist_cmd 에 Twist 퍼블리시
"""

import os, time, math, numpy as np, cv2, torch, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

from berry_interface.action import ServoTwist

# stem_and_yolo: 기존 로직 그대로 사용
from .stem_and_yolo import (
    select_yolo_by_roi_similarity, paint_polygons_with_hsv, get_bottom_mask_polygons,
    BOTTOM_MASK_ENABLE, BOTTOM_MASK_HSV, BOTTOM_MASK_FEATHER
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
        # 카메라 내부 파라미터 (기본값 = 네가 준 값)
        self.declare_parameter("fx", 636.235)
        self.declare_parameter("fy", 619.641)
        self.declare_parameter("cx", 211.89)
        self.declare_parameter("cy", 221.05)

        # 프레임 이름
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("eef_frame",  "link5")
        self.declare_parameter("cam_frame",  "bottom_camera_link")  # bottom 카메라 body frame

        # 하단 카메라 입력
        self.declare_parameter("use_v4l", True)
        self.declare_parameter("v4l_index", 0)
        self.declare_parameter("bottom_rgb_topic", "/bottom_cam/image_raw")

        # YOLO
        self.declare_parameter("yolo_model_rel", "model/model1.pt")
        self.declare_parameter("yolo_conf", 0.01)
        self.declare_parameter("yolo_sim_conf_thresh", 0.30)  # (IoU + conf) 점수 임계값

        # Gripper ROI (bbox x-align 기준)
        self.declare_parameter("grip_roi_xywh", [63, 83, 360, 471])

        # 제어 게인/제약
        # 제어/추론 주기 (같이 맞춤)
        self.declare_parameter("ctrl_rate_hz", 3.0)    # 기본 3 Hz
        # 게인: 카메라 프레임 기준
        self.declare_parameter("K_wx_angle", 0)      # y0–y1 각도(rad) → wx_cam
        self.declare_parameter("K_wz_center", 0.8)     #1.8 (u_err/fx)(rad) → wz_cam
        self.declare_parameter("w_limit", 0.5)         # 각속도 절대 최대 [rad/s]

        self.declare_parameter("ang_tol_deg", 2.0)     # [deg]   종료 기준
        self.declare_parameter("ux_tol_px",   4.0)     # [px]    종료 기준
        self.declare_parameter("steady_count_req", 8)  # 조건 만족 프레임 누적

        # Debug 창
        self.declare_parameter("show_debug", True)

        # ───── 상태 변수 ───────────────────────────────────────────
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)

        self.base_frame = self.get_parameter("base_frame").value
        self.eef_frame  = self.get_parameter("eef_frame").value
        self.cam_frame  = self.get_parameter("cam_frame").value

        self.grip_roi = [int(v) for v in self.get_parameter("grip_roi_xywh").value]

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

        # YOLO 로드
        pkg_share  = get_package_share_directory('berry_perception')
        model_path = os.path.join(pkg_share, self.get_parameter("yolo_model_rel").value)
        self.log.info(f"[YOLO] loading: {model_path}")
        self.yolo = YOLO(model_path)
        self.yolo.model.to('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.yolo_conf = float(self.get_parameter("yolo_conf").value)
        self.sim_conf_thresh = float(self.get_parameter("yolo_sim_conf_thresh").value)

        # TF
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        self._last_tf_warn_sec = 0.0

        # 퍼블리셔 (EEF twist 명령)
        self.twist_pub = self.create_publisher(Twist, "/eef_twist_cmd", 10)

        # ServoTwist 액션 클라이언트
        self.servo_client = ActionClient(self, ServoTwist, "servo_twist")
        self._servo_started = False
        self._requested_stop = False

        # 하단 집게 마스킹 폴리곤
        self._bottom_polys = get_bottom_mask_polygons()

        # 종료 판단 변수
        self._steady_ok = 0

        # 디버그 창
        self.show_debug = bool(self.get_parameter("show_debug").value)
        if self.show_debug:
            cv2.namedWindow("IBVS_RAW",  cv2.WINDOW_NORMAL)
            cv2.namedWindow("IBVS_DBG",  cv2.WINDOW_NORMAL)

        # 제어 루프
        # hz = float(self.get_parameter("ctrl_rate_hz").value)
        # self.timer = self.create_timer(1.0 / max(1e-3, hz), self.tick)
        hz = float(self.get_parameter("ctrl_rate_hz").value)
        self.ctrl_dt = 1.0 / max(1e-3, hz)
        self.timer = self.create_timer(self.ctrl_dt, self.tick)

        self.log.info("▶ IBVSBottomServo started. Will enter servo mode automatically.")

    # ──────────────────────────────────────────────────────────────
    # Callbacks & helpers
    # ──────────────────────────────────────────────────────────────
    def cb_bottom_rgb(self, msg: Image):
        self.bot_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    @staticmethod
    def _xywh_to_xyxy(x, y, w, h):
        return (x, y, x + w, y + h)

    @staticmethod
    def _saturate(x, lim):
        if x >  lim: return  lim
        if x < -lim: return -lim
        return x

    @staticmethod
    def _quat_to_R(qx, qy, qz, qw):
        return tft.quaternion_matrix((qx, qy, qz, qw))[:3, :3]

    def _start_servo_if_needed(self):
        if self._servo_started:
            return
        if not self.servo_client.wait_for_server(timeout_sec=0.0):
            # 아직 서버 준비 안됐으면 다음 tick에 다시 시도
            return
        goal = ServoTwist.Goal()  # 필드가 없어도 OK (정의에 따라 비어있을 수 있음)
        self.servo_client.send_goal_async(goal)
        self._servo_started = True
        self.log.info("[servo] ENTER servo mode (ServoTwist goal sent)")

    def _stop_servo(self):
        """servo 종료 & 속도 0 보장"""
        if not self._requested_stop:
            self._requested_stop = True
            self.log.info("[servo] request STOP")
        # zero twist 1회 더 퍼블리시
        msg = Twist()
        self.twist_pub.publish(msg)

    def _omega_base_from_omega_cam(self, w_cam3: np.ndarray):
        """
        w_cam3: [wx, wy, wz] in bottom camera frame (bottom_camera_link).
        returns: [wx, wy, wz] in base_link frame. None if TF unavailable.
        """
        try:
            tf_base_from_cam = self.tfb.lookup_transform(
                self.base_frame, self.cam_frame, rclpy.time.Time())
            R_base_from_cam = self._quat_to_R(
                tf_base_from_cam.transform.rotation.x,
                tf_base_from_cam.transform.rotation.y,
                tf_base_from_cam.transform.rotation.z,
                tf_base_from_cam.transform.rotation.w)
            # 디버깅: TF rpy
            r, p, y = tft.euler_from_quaternion((
                tf_base_from_cam.transform.rotation.x,
                tf_base_from_cam.transform.rotation.y,
                tf_base_from_cam.transform.rotation.z,
                tf_base_from_cam.transform.rotation.w))
            self.log.info(f"[tf] base<-{self.cam_frame} RPY(deg)=({math.degrees(r):.1f}, {math.degrees(p):.1f}, {math.degrees(y):.1f})")
        except Exception as e:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s - self._last_tf_warn_sec > 1.0:
                self.log.warn(f"[tf] waiting base<-{self.cam_frame}: {e}")
                self.log.warn("      Check frame names or publish static TF. You can set param 'base_frame'.")
                self._last_tf_warn_sec = now_s
            return None
        return (R_base_from_cam @ w_cam3.reshape(3, 1)).reshape(3)
 
    # ──────────────────────────────────────────────────────────────
    # Main control loop
    # ──────────────────────────────────────────────────────────────
    def tick(self):
        # 0) servo 진입 시도
        self._start_servo_if_needed()
        if not self._servo_started:
            return

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

        # (선택) 그리퍼 마스킹 전처리
        if BOTTOM_MASK_ENABLE and len(self._bottom_polys) > 0:
            frame, _ = paint_polygons_with_hsv(
                frame,
                self._bottom_polys,
                hsv_color=BOTTOM_MASK_HSV,
                feather=int(BOTTOM_MASK_FEATHER),
                alpha=1.0
            )

        # 2) YOLO + ROI-유사도 선택  (제어주기와 동일한 1회/틱 = 기본 3Hz)
        gx, gy, gw, gh = self.grip_roi
        gr_xyxy = self._xywh_to_xyxy(gx, gy, gw, gh)
        results = self.yolo.predict(source=frame, conf=self.yolo_conf,
                                    save=False, verbose=False, max_det=10)
        # boxes/conf/kpts 꺼내는 도우미는 여기서 간단히 접근 (results[0].boxes 등)
        xyxy_b = []
        conf_b = []
        if results and results[0].boxes is not None and len(results[0].boxes) > 0:
            b = results[0].boxes
            try:
                arr_xy = b.xyxy.detach().cpu().numpy()
                arr_cf = b.conf.detach().cpu().numpy()
            except Exception:
                arr_xy = b.xyxy.numpy()
                arr_cf = b.conf.numpy()
            xyxy_b = arr_xy.astype(int).tolist()
            conf_b = [float(c) for c in arr_cf.tolist()]

        idx, sim_iou, seg_roi, seg, score = select_yolo_by_roi_similarity(
            bgr=frame,
            gr_xyxy=gr_xyxy,
            xyxy_list=xyxy_b,
            conf_list=conf_b,
            score_thresh=self.sim_conf_thresh
        )
        yolo_valid = (idx is not None)

        # 3) 제어오차 계산  (카메라 프레임 기준 각속도)
        #   (a) y0–y1 각도(세로 기준) → wx_cam
        #   (b) bbox 중심 x vs ROI 중심 x → wz_cam
        wx_cam = 0.0
        wy_cam = 0.0
        wz_cam = 0.0
        ang_err = None
        ux_err  = None

        # ROI 중심 x
        cx_roi = gx + 0.5 * gw

        if yolo_valid:
            x1, y1, x2, y2 = xyxy_b[idx]
            cx_box = 0.5 * (x1 + x2)
            # ux_err = float(cx_box - cx_roi)                 # [px]
            # # (a) wy: u 편차 → 라디안 변환 후 비례제어
            # #     u ≈ fx * θ_y  → θ_y ≈ u/fx
            # theta_y = ux_err / max(1e-6, self.fx)          # [rad]
            # wy_cmd_opt = - float(self.get_parameter("K_wy").value) * theta_y
            ux_err = float(cx_box - cx_roi)                 # [px]
            # (a) wz_cam: u 편차 → θ_z ≈ u/fx
            theta_z = ux_err / max(1e-6, self.fx)          # [rad]
            wz_cam  = - float(self.get_parameter("K_wz_center").value) * theta_z
 
            # (b) wx_cam: y0–y1 라인의 기울기(세로 기준)
            # keypoints 꺼내기 (없으면 skip)
            #   results[0].keypoints.xy shape: [N, K, 2]
            kx0 = ky0 = kx1 = ky1 = None
            rk = getattr(results[0], "keypoints", None)
            if rk is not None and rk.xy is not None:
                try:
                    karr = rk.xy[idx].detach().cpu().numpy()   # (K,2)
                except Exception:
                    karr = rk.xy[idx]
                if karr is not None and len(karr) >= 2:
                    kx0, ky0 = float(karr[0][0]), float(karr[0][1])
                    kx1, ky1 = float(karr[1][0]), float(karr[1][1])

            # if kx0 is not None and kx1 is not None:
            #     # 영상 "세로" 기준 각도: dy 를 기준으로 atan2(dx, dy)
            #     dx = (kx1 - kx0)
            #     dy = (ky1 - ky0)
            #     ang_err = math.atan2(dx, dy)                 # [rad] (세로축 기준)
            #     wz_cmd_opt = - float(self.get_parameter("K_wz").value) * ang_err
            if kx0 is not None and kx1 is not None:
                # 영상 "세로" 기준 각도: dy 를 기준으로 atan2(dx, dy)
                dx = (kx1 - kx0)
                dy = (ky1 - ky0)
                ang_err = math.atan2(dx, dy) + math.pi                 # [rad] (세로축 기준)
                wx_cam  = - float(self.get_parameter("K_wx_angle").value) * ang_err

            # 디버그 오버레이
            if self.show_debug:
                vis = frame.copy()
                # ROI/bbox
                cv2.rectangle(vis, (gx, gy), (gx+gw, gy+gh), (255,255,255), 1)
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.line(vis, (int(cx_roi), 0), (int(cx_roi), vis.shape[0]-1), (255,255,0), 1)
                cv2.circle(vis, (int(cx_box), int(0.5*(y1+y2))), 4, (0,0,255), -1)
                # 키포인트/각도
                if kx0 is not None and kx1 is not None:
                    cv2.circle(vis, (int(kx0), int(ky0)), 4, (255,0,255), -1)
                    cv2.circle(vis, (int(kx1), int(ky1)), 4, (0,165,255), -1)
                    cv2.line(vis, (int(kx0), int(ky0)), (int(kx1), int(ky1)), (0,128,255), 2)
                # 텍스트
                # cv2.putText(vis, f"u_err={ux_err:.1f}px  wy_opt={wy_cmd_opt:.3f}",
                cv2.putText(vis, f"u_err={ux_err:.1f}px  wz_cam={wz_cam:.3f}",
                            (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,255,200), 2)
                if ang_err is not None:
                    # cv2.putText(vis, f"ang_err={math.degrees(ang_err):.2f}deg  wz_opt={wz_cmd_opt:.3f}",
                    cv2.putText(vis, f"ang_err={math.degrees(ang_err):.2f}deg  wx_cam={wx_cam:.3f}",
                                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,255), 2)
                cv2.imshow("IBVS_DBG", vis)
                cv2.imshow("IBVS_RAW", raw)
        else:
            # YOLO 미검출: 안전하게 정지 명령
            wx_cam = wy_cam = wz_cam = 0.0
            if self.show_debug:
                vis = frame.copy()
                cv2.rectangle(vis, (gx, gy), (gx+gw, gy+gh), (255,255,255), 1)
                cv2.putText(vis, f"YOLO invalid (score<=thr={self.sim_conf_thresh:.2f})",
                            (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                cv2.imshow("IBVS_DBG", vis)
                cv2.imshow("IBVS_RAW", raw)

        # # 4) optical → base_link 로 각속도 변환
        # w_opt = np.array([0.0, wy_cmd_opt, wz_cmd_opt], dtype=float)
        # w_base = self._omega_base_from_omega_optical(w_opt)
        # 4) 카메라프레임 → base_link 로 각속도 변환
        w_cam = np.array([wx_cam, wy_cam, wz_cam], dtype=float)
        w_base = self._omega_base_from_omega_cam(w_cam)
        if w_base is None:
            # TF 없으면 정지 퍼블리시 후 다음 tick
            msg = Twist()  # zero
            self.twist_pub.publish(msg)
            return

        # 5) 한계
        w_lim = float(self.get_parameter("w_limit").value)
        # 요구: wx는 카메라축 기준 제어였으나 base로 변환됨 → 그대로 사용
        w_base[1] = self._saturate(w_base[1], w_lim)
        w_base[2] = self._saturate(w_base[2], w_lim)
        w_base[0] = self._saturate(w_base[0], w_lim)

        # 6) 종료 판단 (연속 만족)
        done_now = False
        ang_ok = (ang_err is None) or (abs(math.degrees(ang_err)) <= float(self.get_parameter("ang_tol_deg").value))
        ux_ok  = (ux_err is None) or (abs(ux_err) <= float(self.get_parameter("ux_tol_px").value))
        if ang_ok and ux_ok and yolo_valid:
            self._steady_ok += 1
        else:
            self._steady_ok = 0
        if self._steady_ok >= int(self.get_parameter("steady_count_req").value):
            done_now = True

        # 7) Twist 퍼블리시
        msg = Twist()
        # linear=0
        msg.angular.x = float(w_base[0])
        msg.angular.y = float(w_base[1])
        msg.angular.z = float(w_base[2])
        self.twist_pub.publish(msg)
        self.log.info(
            f"[ibvs] idx={idx if yolo_valid else 'None'} "
            f"score={score:.2f} iou={sim_iou:.2f} "
            f"u_err={ux_err if ux_err is not None else float('nan'):.1f}px "
            f"ang_err={math.degrees(ang_err) if ang_err is not None else float('nan'):.2f}deg | "
            f"w_cam=[{wx_cam:.3f}, {wy_cam:.3f}, {wz_cam:.3f}] rad/s → "
            f"w_base=[{w_base[0]:.3f}, {w_base[1]:.3f}, {w_base[2]:.3f}]"
        )

        # 8) 종료 처리
        if done_now and not self._requested_stop:
            self.log.info("[IBVS] ✔ target reached → stopping servo")
            self._stop_servo()

        # 키보드 종료
        if self.show_debug:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.log.info("⏹ Quit (q)")
                self._stop_servo()
                rclpy.shutdown()

def main():
    rclpy.init()
    node = IBVSBottomServo()
    try:
        rclpy.spin(node)
    finally:
        try: cv2.destroyAllWindows()
        except: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
