# /berry_perception/bottom_cam_param_test.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BottomCamParamTest
 - 상단 RGBD 카메라와 하단 RGB 카메라를 함께 띄워서 클릭 기반으로 대응점 수집
 - 클릭 시 프레임 freeze
 - RGBD 쪽 클릭 → EEF(link5) 기준 3D(gt) 저장
 - RGB 쪽 클릭 → 2D 저장
 - N개 샘플 수집 후, TF로 extrinsic 을 고정하고, 왜곡 0 가정하에 fx, fy만 최소자승으로 추정
   (cx, cy는 영상 중심으로 둠). cv2.calibrateCamera 사용하지 않음.
 - 추정 파라미터로 정합검증: (RGBD, RGB) 두 픽셀만으로 스테레오 삼각측량
   → 3D 비교(정답=RGBD 3D) 및 오차 로그
"""
import os, sys, math, time
from typing import List, Tuple, Optional

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft


class BottomCamParamTest(Node):
    def __init__(self):
        super().__init__("bottom_cam_param_test", automatically_declare_parameters_from_overrides=True)
        self.log = self.get_logger()
        self.bridge = CvBridge()

        # ───────── Parameters ─────────
        # Frames
        self.declare_parameter('eef_frame', 'link5')
        self.declare_parameter('top_cam_frame', 'camera_link')
        self.declare_parameter('bottom_cam_frame', 'bottom_camera_link')

        # Topics (top RGBD)
        self.declare_parameter('top_rgb_topic',   '/camera/camera/color/image_raw')
        self.declare_parameter('top_depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('top_info_topic',  '/camera/camera/color/camera_info')

        # Bottom camera source (topic or v4l)
        self.declare_parameter('bottom_use_v4l', True)
        self.declare_parameter('bottom_v4l_index', 0)
        self.declare_parameter('bottom_rgb_topic', '/bottom_cam/image_raw')

        # Collection & calibration
        self.declare_parameter('num_samples', 25)
        self.declare_parameter('calib_use_distortion', False)  # 보정계수 추정 여부
        self.declare_parameter('show_epipolar', True)          # 에피폴라 선 보조
        # Debugging
        self.declare_parameter('debug_verbose', True)
        self.declare_parameter('debug_dump_dir', '/home/wossas/Desktop/strawberry_robotArm/strawberry_ws/src/berry-good-bot/berry_perception/test/tmp')

        # Internal cfg
        self.num_samples = int(self.get_parameter('num_samples').value)
        self.use_dist = bool(self.get_parameter('calib_use_distortion').value)
        self.show_epi = bool(self.get_parameter('show_epipolar').value)
        self.debug_verbose = bool(self.get_parameter('debug_verbose').value)
        self.debug_dir = str(self.get_parameter('debug_dump_dir').value)
        # ───────── TF ─────────
        self.eef_frame   = self.get_parameter('eef_frame').value
        self.top_frame   = self.get_parameter('top_cam_frame').value
        self.bottom_frame= self.get_parameter('bottom_cam_frame').value
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)

        # ───────── Subscriptions ─────────
        self.top_rgb = None
        self.top_dep = None
        self.top_info: Optional[CameraInfo] = None
        self.create_subscription(Image, self.get_parameter('top_rgb_topic').value, self.cb_top_rgb, 10)
        self.create_subscription(Image, self.get_parameter('top_depth_topic').value, self.cb_top_depth, 10)
        self.create_subscription(CameraInfo, self.get_parameter('top_info_topic').value, self.cb_top_info, 10)

        # Bottom source
        self.bot_img = None
        self.bot_cap = None
        if bool(self.get_parameter('bottom_use_v4l').value):
            idx = int(self.get_parameter('bottom_v4l_index').value)
            self.bot_cap = cv2.VideoCapture(idx)
            if not self.bot_cap.isOpened():
                self.log.error(f"[bottom] cannot open VideoCapture index {idx}")
                self.bot_cap = None
        else:
            self.create_subscription(Image, self.get_parameter('bottom_rgb_topic').value, self.cb_bot_rgb, 10)

        # ───────── Windows & Mouse ─────────
        self.win_top = "TOP_RGBD"
        self.win_bot = "BOTTOM_RGB"
        cv2.namedWindow(self.win_top, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.win_bot, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.win_top,  self.on_click_top)
        cv2.setMouseCallback(self.win_bot,  self.on_click_bottom)

        # ───────── State ─────────
        self.state = 'WAIT_TOP'   # WAIT_TOP -> WAIT_BOTTOM -> (repeat) -> DONE
        self.freeze = False
        self.last_top_bgr = None
        self.last_bot_bgr = None
        self.cur_top_uv: Optional[Tuple[int,int]] = None
        self.cur_bot_uv: Optional[Tuple[int,int]] = None
        self.cur_top_gt_eef: Optional[np.ndarray] = None  # 3D (EEF)

        # Data arrays
        self.samples_obj_eef: List[np.ndarray] = []  # Nx3 float32 (EEF)
        self.samples_uv_top:  List[Tuple[float,float]] = []  # (u,v)
        self.samples_uv_bot:  List[Tuple[float,float]] = []  # (u,v)
        self.debug_overlay = True  # toggle with 'd'

        # Calibration result (bottom)
        self.K_bot = None
        self.D_bot = None
        self.R_bot = None  # world(EEF)->bottom R
        self.t_bot = None  # world(EEF)->bottom t
        self._log_tf_snapshot_once()

        # Timer for UI
        self.timer = self.create_timer(1.0/20.0, self.tick)
        self.log.info("▶ BottomCamParamTest started.")
        self.log.info("   - TOP window: click a strawberry point (RGBD). Frames freeze.")
        self.log.info("   - BOT window: click the corresponding point (RGB).")
        self.log.info(f"   - Repeat {self.num_samples} times → auto-calibration & validation.")

    # ───────────────────────── Callbacks ─────────────────────────
    def cb_top_rgb(self, msg: Image):
        if self.freeze:
            return
        self.top_rgb = msg

    def cb_top_depth(self, msg: Image):
        if self.freeze:
            return
        self.top_dep = msg

    def cb_top_info(self, msg: CameraInfo):
        self.top_info = msg

    def cb_bot_rgb(self, msg: Image):
        if self.freeze:
            return
        self.bot_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # ───────────────────────── Mouse ─────────────────────────
    def on_click_top(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.state != 'WAIT_TOP':
            return
        if self.top_rgb is None or self.top_dep is None or self.top_info is None:
            # self.log.warn("TOP not ready yet.")
            self.log.warning("TOP not ready yet.")
            return
        # Freeze frames
        self.freeze = True
        # Hold current BGR frames
        self.last_top_bgr = self.bridge.imgmsg_to_cv2(self.top_rgb, 'bgr8').copy()
        if self.bot_cap is not None:
            ok, b = self.bot_cap.read()
            if ok: self.last_bot_bgr = b.copy()
        else:
            self.last_bot_bgr = None if self.bot_img is None else self.bot_img.copy()

        # Compute GT 3D (EEF) from RGBD click
        depth_np = self.bridge.imgmsg_to_cv2(self.top_dep, 'passthrough').astype(np.float32) / 1000.0
        d = self._robust_depth_at(x, y, depth_np)
        if not np.isfinite(d) or d <= 0.0:
            # self.log.warn(f"[TOP] invalid depth at ({x},{y})")
            self.log.warning(f"[TOP] invalid depth at ({x},{y})")
            self.freeze = False
            return
        Kt, Dt = self._K_from_info(self.top_info)
        # undistort to normalized coords
        pt = np.array([[[float(x), float(y)]]], dtype=np.float32)
        uvn = cv2.undistortPoints(pt, Kt, Dt, P=None)  # → normalized coords (x',y')
        xn, yn = float(uvn[0,0,0]), float(uvn[0,0,1])
        # # optical frame → 3D cam coords (z=depth)
        # Xc_opt = np.array([xn * d, yn * d, d], dtype=np.float64)
        xn, yn = float(uvn[0,0,0]), float(uvn[0,0,1])
        # optical frame → 3D cam coords (z=depth)  → 열벡터(3,1)로 유지
        Xc_opt = np.array([[xn * d],
                           [yn * d],
                           [d       ]], dtype=np.float64)
        # cam(optical) → top_cam_frame? 대부분 optical과 camera_link가 다름.
        # 여기서는 top_cam_frame이 camera_link(Body: X=+z,Y=-x,Z=-y)인 이전 코드와의 호환을 피하려고
        # TF에서 직접 변환을 구한다: (world=eef) ← top_camera_optical
        eef = self.eef_frame
        # 우선 world(EEF) ← top_cam_optical 변환을 얻는다. 불가능하면 world←top_cam_link 후 optical 보정 적용 필요.
        # 안전하게: world ← top_cam_frame (camera_link) 변환을 사용하고, optical→link 고정 회전 적용.
        try:
            # world ← top_cam_frame
            T_w_c = self._tf_eef_from(self.top_frame)
            R_w_c, t_w_c = T_w_c
            # # optical→camera_link 보정 (고정): X_link=+Z_opt, Y_link=-X_opt, Z_link=-Y_opt
            # R_link_opt = np.array([[0, -1, 0],
            #                        [0,  0,-1],
            #                        [1,  0, 0]], dtype=np.float64)  # maps opt→link

            # optical→camera_link 보정 (고정):
            #   X_link=+Z_opt, Y_link=-X_opt, Z_link=-Y_opt
            #   (OpenCV optical: x=right, y=down, z=forward)
            R_link_opt = np.array([[ 0,  0,  1],
                                   [-1,  0,  0],
                                   [ 0, -1,  0]], dtype=np.float64)  # maps opt→link


            Xc_link = R_link_opt @ Xc_opt                 # (3,1)
            Xw_col  = R_w_c @ Xc_link + t_w_c             # (3,1)
            Xw      = Xw_col.reshape(3)                   # (3,)
        except Exception as e:
            self.log.error(f"TF lookup (top) failed: {e}")
            self.freeze = False
            return

        self.cur_top_uv = (int(x), int(y))
        self.cur_top_gt_eef = Xw  # 3D in EEF

        # Draw marker on frozen images & (optional) epipolar line on bottom
        self._overlay_after_top_click()
        self.state = 'WAIT_BOTTOM'
        self.log.info(f"[{len(self.samples_obj_eef)+1}/{self.num_samples}] Click the SAME point on BOTTOM window.")

    def on_click_bottom(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.state != 'WAIT_BOTTOM':
            return
        if self.cur_top_uv is None or self.cur_top_gt_eef is None:
            return
        self.cur_bot_uv = (int(x), int(y))

        # # Save one sample
        # self.samples_obj_eef.append(self.cur_top_gt_eef.astype(np.float32))
        # (3,) 형태로 고정하여 저장
        self.samples_obj_eef.append(self.cur_top_gt_eef.reshape(3).astype(np.float32))
        self.samples_uv_top.append( (float(self.cur_top_uv[0]),  float(self.cur_top_uv[1])) )
        self.samples_uv_bot.append( (float(self.cur_bot_uv[0]), float(self.cur_bot_uv[1])) )

        # self.log.info(f"[ADD] sample #{len(self.samples_obj_eef)} stored.")

        idx = len(self.samples_obj_eef)
        self.log.info(f"[ADD] sample #{idx} stored.")
        # Debug: print pair + predicted bottom pixel from TF (using crude K if unknown)
        if self.debug_verbose:
            try:
                Kt, Dt = self._K_from_info(self.top_info) if self.top_info is not None else (None, None)
                if self.last_bot_bgr is not None:
                    h, w = self.last_bot_bgr.shape[:2]
                    if self.K_bot is None:
                        f_guess = float(max(w, h))
                        Kb_dbg = np.array([[f_guess, 0, w*0.5],[0, f_guess, h*0.5],[0,0,1.0]], np.float64)
                    else:
                        Kb_dbg = self.K_bot
                # world→bottom(opt)
                R_b_w_link, t_b_w_link = self._tf_eef_from(self.bottom_frame)
                R_w2b_link, t_w2b_link = self._invert_rt(R_b_w_link, t_b_w_link)
                R_link_opt = np.array([[0,0,1],[-1,0,0],[0,-1,0]], np.float64)
                R_w2b = R_link_opt.T @ R_w2b_link
                t_w2b = R_link_opt.T @ t_w2b_link
                Xw = self.cur_top_gt_eef.reshape(3,1).astype(np.float64)
                Xc = R_w2b @ Xw + t_w2b
                # u_pred = float((Kb_dbg @ np.vstack([Xc, [1.0]]))[:2].ravel()[0] / Xc[2,0]) if Xc[2,0]!=0 else np.nan
                # v_pred = float((Kb_dbg @ np.vstack([Xc, [1.0]]))[:2].ravel()[1] / Xc[2,0]) if Xc[2,0]!=0 else np.nan
                if Xc[2,0] != 0:
                    uvh = Kb_dbg @ (Xc / Xc[2,0])
                    u_pred, v_pred = float(uvh[0,0]), float(uvh[1,0])
                else:
                    u_pred, v_pred = float('nan'), float('nan')

                self.log.info(f"    • pair{idx}: TOP uv={self.samples_uv_top[-1]}, BOT uv={self.samples_uv_bot[-1]}, "
                              f"EEF={self.cur_top_gt_eef}, bottom Z={Xc[2,0]:.4f} m, pred_bot≈({u_pred:.1f},{v_pred:.1f})")
                # epipolar distance
                if self.top_info is not None and self.last_bot_bgr is not None:
                    # R_t_w, t_t_w = self._tf_eef_from(self.top_frame)
                    # R_w2t, t_w2t = self._invert_rt(R_t_w, t_t_w)
                    # F = self._compute_F(Kt, Kb_dbg, R_w2t, t_w2t, R_w2b, t_w2b)
                    # world→top_optical (link→optical 보정 적용)
                    R_t_w, t_t_w = self._tf_eef_from(self.top_frame)       # child(top_link)→EEF
                    R_w2t_link, t_w2t_link = self._invert_rt(R_t_w, t_t_w) # world→top_link
                    R_opt_link = R_link_opt.T
                    R_w2t_opt  = R_opt_link @ R_w2t_link
                    t_w2t_opt  = R_opt_link @ t_w2t_link
                    F = self._compute_F(Kt, Kb_dbg, R_w2t_opt, t_w2t_opt, R_w2b, t_w2b)

                    d_epi = self._point_line_dist(F, self.samples_uv_top[-1], self.samples_uv_bot[-1])
                    self.log.info(f"    • epipolar dist (px): {d_epi:.2f}")
            except Exception as e:
                self.log.warning(f"    • debug pair compute failed: {e}")
 
        # Visualize on frozen bottom frame
        if self.last_bot_bgr is not None:
            cv2.circle(self.last_bot_bgr, (int(x),int(y)), 5, (0,255,0), -1)
            cv2.putText(self.last_bot_bgr, f"{len(self.samples_obj_eef)}", (int(x)+6, max(0,int(y)-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # Reset for next sample or run calibration
        self.cur_top_uv = None
        self.cur_bot_uv = None
        self.cur_top_gt_eef = None
        self.freeze = False
        self.state = 'WAIT_TOP'

        if len(self.samples_obj_eef) >= self.num_samples:
            self.state = 'DONE'
            self._run_calibration_and_validate()


    # ───────────────────────── Small math helpers ─────────────────────────
    @staticmethod
    def _invert_rt(R: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Given X_w → X_c : X_c = R*X_w + t  (world→camera),
        if input (R,t) is camera→world (X_w = R*X_c + t), return its inverse.
        """
        Rcw = R.T; tcw = -Rcw @ t.reshape(3,1); return Rcw, tcw

    # ───────────────────────── Timed UI tick ─────────────────────────
    def tick(self):
        # Update frames unless frozen
        if not self.freeze:
            if self.top_rgb is not None:
                self.last_top_bgr = self.bridge.imgmsg_to_cv2(self.top_rgb, 'bgr8')
            if self.bot_cap is not None:
                ok, frm = self.bot_cap.read()
                if ok: self.last_bot_bgr = frm
            else:
                if self.bot_img is not None:
                    self.last_bot_bgr = self.bot_img

        # Draw instructions
        disp_top = None if self.last_top_bgr is None else self.last_top_bgr.copy()
        disp_bot = None if self.last_bot_bgr is None else self.last_bot_bgr.copy()
        if disp_top is not None:
            cv2.putText(disp_top, f"[{self.state}] Click a point here (TOP RGBD) first.",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(disp_top, f"Samples: {len(self.samples_obj_eef)}/{self.num_samples}",
                        (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
            cv2.putText(disp_top, "[q] quit  [r] reset  [d] overlay  [s] save", (10, 78),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180,180,180), 1)
        if disp_bot is not None:
            cv2.putText(disp_bot, f"[{self.state}] Then click the SAME point here (BOTTOM RGB).",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            if self.debug_overlay:
                try:
                    self._draw_all_reproj(disp_bot)
                except Exception as e:
                    self.log.warning(f"debug overlay failed: {e}")

        # Show
        if disp_top is not None: cv2.imshow(self.win_top, disp_top)
        if disp_bot is not None: cv2.imshow(self.win_bot, disp_bot)

        # Handle key
        if (disp_top is not None) or (disp_bot is not None):
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.log.info("⏹ Quit requested (q)")
                rclpy.shutdown()
            elif key == ord('r'):
                self._reset_samples()
                self.log.info("↺ Samples reset.")
            elif key == ord('d'):
                self.debug_overlay = not self.debug_overlay
                self.log.info(f"🧪 Debug overlay: {'ON' if self.debug_overlay else 'OFF'}")
            elif key == ord('s'):
                try:
                    ts = int(time.time())
                    if disp_bot is not None:
                        path = os.path.join(self.debug_dir, f"bottom_debug_{ts}.png")
                        cv2.imwrite(path, disp_bot)
                        self.log.info(f"💾 Saved debug image: {path}")
                except Exception as e:
                    self.log.warning(f"save failed: {e}")

    # ───────────────────────── Helpers ─────────────────────────
    @staticmethod
    def _K_from_info(info: CameraInfo):
        K = np.array(info.k, dtype=np.float64).reshape(3,3)
        D = np.array(info.d, dtype=np.float64).reshape(-1,1) if info.d else np.zeros((5,1), np.float64)
        return K, D

    @staticmethod
    def _robust_depth_at(u_px: int, v_px: int, depth_np: np.ndarray) -> float:
        """주변 3x3에서 하위 25% 평균"""
        h, w = depth_np.shape[:2]
        u = max(0, min(w-1, int(round(u_px))))
        v = max(0, min(h-1, int(round(v_px))))
        x0, x1 = max(0,u-1), min(w, u+2)
        y0, y1 = max(0,v-1), min(h, v+2)
        win = depth_np[y0:y1, x0:x1].reshape(-1)
        vals = win[(win > 0) & np.isfinite(win)]
        if vals.size == 0:
            return float('nan')
        s = np.sort(vals)
        k = max(1, int(len(s)*0.25))
        return float(np.mean(s[:k]))

    def _tf_eef_from(self, child_frame: str):
        """
        Returns (R, t) such that: X_eef = R * X_child + t
        (i.e., transform child → EEF/world)
       """
        tf = self.tfb.lookup_transform(self.eef_frame, child_frame, rclpy.time.Time())
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z], dtype=np.float64).reshape(3,1)
        q = tf.transform.rotation
        R = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3,:3].astype(np.float64)
        return R, t

    @staticmethod
    def _proj_from_rt(K: Optional[np.ndarray], R: np.ndarray, t: np.ndarray) -> np.ndarray:
        Rt = np.hstack([R, t.reshape(3,1)])
        if K is None:
            return Rt
        return K @ Rt

    @staticmethod
    def _skew(v: np.ndarray) -> np.ndarray:
        x,y,z = v.ravel()
        return np.array([[0,-z, y],[z,0,-x],[-y,x,0]], dtype=np.float64)

    def _epi_line_on_bottom(self, top_uv: Tuple[int,int], img_bot: np.ndarray,
                            Kt: np.ndarray, Kb: np.ndarray,
                            R1: np.ndarray, t1: np.ndarray,
                            R2: np.ndarray, t2: np.ndarray):
        """
        Draw epipolar line on bottom given a top pixel.
        Assumes R1,t1 and R2,t2 are **world→camera** extrinsics for TOP and BOTTOM.
        """
        if img_bot is None or Kb is None or Kt is None:
            return
        # relative pose c2 ← c1
        R_rel = R2 @ R1.T
        t_rel = (t2 - R_rel @ t1).reshape(3,1)
        E = self._skew(t_rel) @ R_rel  # 3x3
        try:
            Kb_invT = np.linalg.inv(Kb).T
            Kt_inv  = np.linalg.inv(Kt)
        except np.linalg.LinAlgError:
            return
        F = Kb_invT @ E @ Kt_inv
        x1 = np.array([top_uv[0], top_uv[1], 1.0], dtype=np.float64).reshape(3,1)
        l2 = F @ x1  # line ax + by + c = 0 in bottom image
        a,b,c = l2.ravel()
        h, w = img_bot.shape[:2]
        # two points on the line within image bounds
        pts = []
        # x=0 → y=-c/b
        if abs(b) > 1e-9:
            y0 = int(round(-c/b))
            if 0 <= y0 < h: pts.append((0, y0))
        # x=w-1 → y=-(a*w+c)/b
        if abs(b) > 1e-9:
            y1 = int(round(-(a*(w-1)+c)/b))
            if 0 <= y1 < h: pts.append((w-1, y1))
        # y=0 → x=-c/a
        if abs(a) > 1e-9:
            x0 = int(round(-c/a))
            if 0 <= x0 < w: pts.append((x0, 0))
        # y=h-1 → x=-(b*h+c)/a
        if abs(a) > 1e-9:
            x1p = int(round(-(b*(h-1)+c)/a))
            if 0 <= x1p < w: pts.append((x1p, h-1))
        # draw with two farthest points
        if len(pts) >= 2:
            # pick two with max distance
            dmax, pair = -1, (pts[0], pts[1])
            for i in range(len(pts)):
                for j in range(i+1, len(pts)):
                    d = (pts[i][0]-pts[j][0])**2 + (pts[i][1]-pts[j][1])**2
                    if d > dmax:
                        dmax, pair = d, (pts[i], pts[j])
            cv2.line(img_bot, pair[0], pair[1], (0,255,255), 1, cv2.LINE_AA)
            cv2.putText(img_bot, "EPI", (pair[0][0]+5, max(0,pair[0][1]-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

    def _overlay_after_top_click(self):
        """Freeze된 프레임에 TOP 클릭 지점과 (가능하면) BOT 에피폴라 라인 표시."""
        if self.last_top_bgr is None or self.cur_top_uv is None:
            return
        u,v = self.cur_top_uv
        cv2.circle(self.last_top_bgr, (u,v), 6, (0,0,255), -1)
        cv2.putText(self.last_top_bgr, "TOP pick", (u+8, max(0,v-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

        if not self.show_epi or self.last_bot_bgr is None:
            return
        # Try to compute an epipolar line on bottom using current best parameters
        if self.top_info is None:
            return
        Kt, Dt = self._K_from_info(self.top_info)
        # Use calibrated K_bot if available, otherwise a crude guess
        if self.K_bot is None:
            h, w = self.last_bot_bgr.shape[:2]
            f_guess = float(max(w,h))  # crude
            Kb = np.array([[f_guess, 0, w*0.5],
                           [0, f_guess, h*0.5],
                           [0, 0, 1.0]], dtype=np.float64)
        else:
            Kb = self.K_bot
        # TOP extrinsic from TF (world=EEF → top)
        try:
            # world→top_optical (link→optical 보정 적용)
            R_t_w, t_t_w = self._tf_eef_from(self.top_frame)       # child(top_link)→EEF
            R_w2t_link, t_w2t_link = self._invert_rt(R_t_w, t_t_w) # world→top_link
            R_link_opt = np.array([[ 0,  0,  1],
                                   [-1,  0,  0],
                                   [ 0, -1,  0]], dtype=np.float64)
            R_opt_link = R_link_opt.T
            R_w2t = R_opt_link @ R_w2t_link
            t_w2t = R_opt_link @ t_w2t_link
        except Exception:
            return
        # BOTTOM extrinsic: prefer estimated (optical). Else build from TF (link→optical 보정 포함)
        try:
            R_b_w_link, t_b_w_link = self._tf_eef_from(self.bottom_frame)   # child(link)→EEF(world)
            R_w2b_link, t_w2b_link = self._invert_rt(R_b_w_link, t_b_w_link) # world→bottom_link
            # optical ↔ link 고정 회전 (opt→link). world→optical 로 변환.
            R_link_opt = np.array([[ 0,  0,  1],
                                   [-1,  0,  0],
                                   [ 0, -1,  0]], dtype=np.float64)        # opt→link

            R_w2b_tf  = R_link_opt.T @ R_w2b_link
            t_w2b_tf  = R_link_opt.T @ t_w2b_link
        except Exception:
            return
        # if self.R_bot is not None and self.t_bot is not None:
        #     R_w_b, t_w_b = self.R_bot, self.t_bot.reshape(3,1)
        # self._epi_line_on_bottom((u,v), self.last_bot_bgr, Kt, Kb, R_w_t, t_w_t, R_w_b, t_w_b)
        # Prefer calibrated bottom extrinsics if available (already world→camera)
        if self.R_bot is not None and self.t_bot is not None:
            R_w2b, t_w2b = self.R_bot, self.t_bot.reshape(3,1)
        else:
            R_w2b, t_w2b = R_w2b_tf, t_w2b_tf

        self._epi_line_on_bottom((u,v), self.last_bot_bgr, Kt, Kb, R_w2t, t_w2t, R_w2b, t_w2b)
 

    # ───────────────────────── Calibration & Validation ─────────────────────────
    def _run_calibration_and_validate(self):
        self.log.info("▶ Estimating fx, fy (no distortion) using TF extrinsic ...")
        if len(self.samples_obj_eef) < 3:
            self.log.error("Not enough samples (need ≥3).")
            return
        if self.last_bot_bgr is None:
            self.log.error("Bottom image not available for size.")
            return
        # 1) world(EEF)→bottom_optical extrinsic from TF (bottom_link→optical 보정 포함)
        try:
            R_b_w_link, t_b_w_link = self._tf_eef_from(self.bottom_frame)  # child(link)→EEF(world)
            R_w2b_link, t_w2b_link = self._invert_rt(R_b_w_link, t_b_w_link)  # world→bottom_link
            R_link_opt = np.array([[ 0,  0,  1],
                                   [-1,  0,  0],
                                   [ 0, -1,  0]], dtype=np.float64)  # opt→link
            R_w2b = R_link_opt.T @ R_w2b_link
            t_w2b = R_link_opt.T @ t_w2b_link
        except Exception as e:
            self.log.error(f"TF (EEF←BOTTOM) failed: {e}")
            return
        
        # 2) Assemble per-point normalized slopes x/z, y/z
        h, w = self.last_bot_bgr.shape[:2]
        
        cx0, cy0 = w * 0.5, h * 0.5  # for reference (초기 직감)
        obj = np.stack(self.samples_obj_eef, axis=0).astype(np.float64).reshape(-1,3)  # (N,3)
        uv  = np.stack(self.samples_uv_bot, axis=0).astype(np.float64).reshape(-1,2)   # (N,2)
        Xw  = obj.T  # (3,N)
        Xc  = (R_w2b @ Xw) + t_w2b  # (3,N)
        x, y, z = Xc[0,:], Xc[1,:], Xc[2,:]

        # optical 기준: z>0 (전방)만 사용
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > 1e-9)
        if self.debug_verbose:
            nz = np.count_nonzero(z <= 0)
            self.log.info(f"[DBG] bottom Xc z-stats: min={np.min(z):.4f}, max={np.max(z):.4f}, "
                          f"neg_or_zero={nz}/{z.size}")

        if np.count_nonzero(valid) < 3:
            self.log.error("Valid sample count too small after filtering.")
            return
        # s = (x[valid] / z[valid])  # x/z
        # t = (y[valid] / z[valid])  # y/z
        # du = (uv[valid,0] - cx)
        # dv = (uv[valid,1] - cy)
        # # 3) Least-squares (zero-intercept): fx = argmin ||fx*s - du||^2, fy likewise
        # fx = float(np.dot(s, du) / max(1e-12, np.dot(s, s)))
        # fy = float(np.dot(t, dv) / max(1e-12, np.dot(t, t)))

        s = (x[valid] / z[valid])  # x/z
        t = (y[valid] / z[valid])  # y/z
        u_obs = uv[valid,0]
        v_obs = uv[valid,1]
        # 3) Least-squares for [fx, cx] and [fy, cy]
        A_u = np.vstack([s, np.ones_like(s)]).T  # (N,2)
        A_v = np.vstack([t, np.ones_like(t)]).T
        theta_u, *_ = np.linalg.lstsq(A_u, u_obs, rcond=None)  # [fx, cx]
        theta_v, *_ = np.linalg.lstsq(A_v, v_obs, rcond=None)  # [fy, cy]
        fx, cx = float(theta_u[0]), float(theta_u[1])
        fy, cy = float(theta_v[0]), float(theta_v[1])

        # 4) Save parameters
        K = np.array([[fx, 0.0, cx],
                      [0.0, fy, cy],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
        self.K_bot = K
        self.D_bot = np.zeros((5,1), np.float64)  # no distortion
        self.R_bot = R_w2b
        self.t_bot = t_w2b.reshape(3,1)
        # 5) Report residuals
        reproj = np.vstack([fx * s + cx, fy * t + cy]).T  # (N_valid,2)
        err_px = np.linalg.norm(reproj - uv[valid,:], axis=1)
        # self.log.info(f"[EST] fx = {fx:.6f}, fy = {fy:.6f}, cx = {cx:.3f}, cy = {cy:.3f} (pixels)")
        
        self.log.info(f"[EST] fx = {fx:.3f}, fy = {fy:.3f}, cx = {cx:.2f}, cy = {cy:.2f} (px)")
        self.log.info(f"[EST] center(W/2,H/2)=({w*0.5:.2f},{h*0.5:.2f}),  Δc=({cx-w*0.5:+.2f}, {cy-h*0.5:+.2f}) px")

        self.log.info(f"[EST] mean reproj error = {np.mean(err_px):.4f} px, median = {np.median(err_px):.4f} px, max = {np.max(err_px):.4f} px")
        # Epipolar distance stats with final K
        try:
            if self.top_info is not None:
                Kt, Dt = self._K_from_info(self.top_info)
                # R_t_w, t_t_w = self._tf_eef_from(self.top_frame)
                # R_w2t, t_w2t = self._invert_rt(R_t_w, t_t_w)

                # world→top_optical
                R_t_w, t_t_w = self._tf_eef_from(self.top_frame)
                R_w2t_link, t_w2t_link = self._invert_rt(R_t_w, t_t_w)
                R_link_opt = np.array([[ 0,  0,  1],
                                       [-1,  0,  0],
                                       [ 0, -1,  0]], dtype=np.float64)
                R_opt_link = R_link_opt.T
                R_w2t, t_w2t = R_opt_link @ R_w2t_link, R_opt_link @ t_w2t_link
                F = self._compute_F(Kt, self.K_bot, R_w2t, t_w2t, self.R_bot, self.t_bot)
                dists = [self._point_line_dist(F, self.samples_uv_top[i], self.samples_uv_bot[i])
                         for i in range(len(self.samples_uv_top))]
                self.log.info(f"[EPI] mean={np.mean(dists):.2f}px, median={np.median(dists):.2f}px, max={np.max(dists):.2f}px")
        except Exception as e:
            self.log.warning(f"[EPI] stat failed: {e}")

        # Validation via triangulation (stereo with TOP & BOTTOM)
        self._validate_triangulation()
        self.log.info("▶ Done. Press 'q' to exit or continue collecting (window active).")
        self.state = 'WAIT_TOP'
        self.freeze = False
        

    def _validate_triangulation(self):
        if self.top_info is None or self.K_bot is None or self.R_bot is None:
            self.log.error("Validation prerequisites missing.")
            return
        Kt, Dt = self._K_from_info(self.top_info)
        # Extrinsics world(EEF) → cams

        try:
            # world→top_optical (link→optical 보정 적용)
            R_t_w, t_t_w = self._tf_eef_from(self.top_frame)       # child(top_link)→EEF
            R_w2t_link, t_w2t_link = self._invert_rt(R_t_w, t_t_w) # world→top_link
            R_link_opt = np.array([[ 0,  0,  1],
                                   [-1,  0,  0],
                                   [ 0, -1,  0]], dtype=np.float64)
            R_opt_link = R_link_opt.T
            R_w2t, t_w2t = R_opt_link @ R_w2t_link, R_opt_link @ t_w2t_link
        except Exception as e:
            self.log.error(f"TF (EEF←TOP) failed: {e}")
            return
        
        # Bottom extrinsics are already world→camera (optical) from TF+link→opt 보정
        R_w2b, t_w2b = self.R_bot, self.t_bot

        # Projection matrices without K (use normalized coords)
        P1 = self._proj_from_rt(None, R_w2t, t_w2t)  # 3x4 (world→top)
        P2 = self._proj_from_rt(None, R_w2b, t_w2b)  # 3x4 (world→bottom)
        errs = []
        self.log.info("── Validation (per-point) ─────────────────────────────────────────")
        for i in range(len(self.samples_obj_eef)):
            u1, v1 = self.samples_uv_top[i]
            u2, v2 = self.samples_uv_bot[i]
            # undistort to normalized coords
            pt1 = np.array([[[u1, v1]]], dtype=np.float32)
            uv1n = cv2.undistortPoints(pt1, Kt, Dt, P=None).reshape(2)
            if self.D_bot is not None and np.any(np.abs(self.D_bot)>1e-12):
                pt2 = np.array([[[u2, v2]]], dtype=np.float32)
                uv2n = cv2.undistortPoints(pt2, self.K_bot, self.D_bot, P=None).reshape(2)
            else:
                # no distortion → normalize by K
                uv2n = (np.linalg.inv(self.K_bot) @ np.array([u2, v2, 1.0], dtype=np.float64)).reshape(3)
                uv2n = uv2n[:2] / uv2n[2]
            # triangulate (expects 2xN arrays)
            x1 = np.array([[uv1n[0]], [uv1n[1]]], dtype=np.float64)
            x2 = np.array([[uv2n[0]], [uv2n[1]]], dtype=np.float64)
            X_h = cv2.triangulatePoints(P1, P2, x1, x2)  # 4xN
            X = (X_h[:3] / X_h[3]).reshape(3)
            gt = self.samples_obj_eef[i].reshape(3)
            err = float(np.linalg.norm(X - gt))
            errs.append(err)
            self.log.info(f"[{i+1:02d}] GT(eef)={gt}, Triang={X}, |err|={err:.4f} m")
            if self.debug_verbose:
                # also check depth sign in each cam
                Zt = (R_w2t @ X.reshape(3,1) + t_w2t)[2,0]
                Zb = (R_w2b @ X.reshape(3,1) + t_w2b)[2,0]
                self.log.info(f"     depth_sign: top Z={Zt:+.4f}, bottom Z={Zb:+.4f}")

        if errs:
            self.log.info(f"── Mean err = {np.mean(errs):.4f} m,  Median = {np.median(errs):.4f} m,  Max = {np.max(errs):.4f} m")


    # ───────────────────────── Debug helpers ─────────────────────────
    def _draw_all_reproj(self, img_bot: np.ndarray):
        """Draw observed (green) vs reprojected (magenta) with error arrows on bottom frame."""
        if img_bot is None or len(self.samples_obj_eef) == 0:
            return
        h, w = img_bot.shape[:2]
        if self.K_bot is None:
            f_guess = float(max(w, h))
            Kb = np.array([[f_guess,0,w*0.5],[0,f_guess,h*0.5],[0,0,1.0]], np.float64)
        else:
            Kb = self.K_bot
        # world→bottom(opt)
        R_b_w_link, t_b_w_link = self._tf_eef_from(self.bottom_frame)
        R_w2b_link, t_w2b_link = self._invert_rt(R_b_w_link, t_b_w_link)
        R_link_opt = np.array([[0,0,1],[-1,0,0],[0,-1,0]], np.float64)
        R_w2b = self.R_bot if self.R_bot is not None else (R_link_opt.T @ R_w2b_link)
        t_w2b = self.t_bot if self.t_bot is not None else (R_link_opt.T @ t_w2b_link)
        for i in range(len(self.samples_obj_eef)):
            Xw = self.samples_obj_eef[i].reshape(3,1).astype(np.float64)
            Xc = R_w2b @ Xw + t_w2b
            if Xc[2,0] <= 1e-9:
                continue
            uvh = Kb @ (Xc / Xc[2,0])
            up, vp = int(round(uvh[0,0])), int(round(uvh[1,0]))
            uo, vo = int(round(self.samples_uv_bot[i][0])), int(round(self.samples_uv_bot[i][1]))
            # observed
            cv2.circle(img_bot, (uo,vo), 5, (0,255,0), -1)
            # predicted
            cv2.circle(img_bot, (up,vp), 4, (255,0,255), 1)
            # error arrow
            cv2.arrowedLine(img_bot, (uo,vo), (up,vp), (255,0,255), 1, tipLength=0.25)
            cv2.putText(img_bot, f"{i+1}", (uo+6, max(0,vo-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        # draw principal point crosshair if available
        if self.K_bot is not None:
            cx_pp, cy_pp = int(round(self.K_bot[0,2])), int(round(self.K_bot[1,2]))
            cv2.drawMarker(img_bot, (cx_pp,cy_pp), (0,255,255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=1)
            cv2.putText(img_bot, "cx,cy", (cx_pp+6, max(0,cy_pp-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

    def _compute_F(self, Kt: np.ndarray, Kb: np.ndarray,
                   R_w2t: np.ndarray, t_w2t: np.ndarray,
                   R_w2b: np.ndarray, t_w2b: np.ndarray) -> np.ndarray:
        """Fundamental matrix from world→cam extrinsics."""
        R_rel = R_w2b @ R_w2t.T
        t_rel = (t_w2b - R_rel @ t_w2t).reshape(3,1)
        E = self._skew(t_rel) @ R_rel
        F = np.linalg.inv(Kb).T @ E @ np.linalg.inv(Kt)
        return F

    @staticmethod
    def _point_line_dist(F: np.ndarray, uv_top: Tuple[float,float], uv_bot: Tuple[float,float]) -> float:
        """Distance from bottom point to epipolar line induced by top point."""
        x1 = np.array([uv_top[0], uv_top[1], 1.0], dtype=np.float64).reshape(3,1)
        l2 = F @ x1
        a,b,c = l2.ravel()
        u2,v2 = uv_bot
        return float(abs(a*u2 + b*v2 + c) / max(1e-9, math.hypot(a,b)))

    def _log_tf_snapshot_once(self):
        """One-shot TF summary: baselines and eulers."""
        try:
            R_t_w, t_t_w = self._tf_eef_from(self.top_frame)
            R_b_w, t_b_w = self._tf_eef_from(self.bottom_frame)
            # child→EEF (cam_link). Build world→opt for bottom
            R_w2t, t_w2t = self._invert_rt(R_t_w, t_t_w)
            R_w2b_link, t_w2b_link = self._invert_rt(R_b_w, t_b_w)
            R_link_opt = np.array([[0,0,1],[-1,0,0],[0,-1,0]], np.float64)
            R_w2b = R_link_opt.T @ R_w2b_link
            t_w2b = R_link_opt.T @ t_w2b_link
            # relative pose (b ← t)
            R_rel = R_w2b @ R_w2t.T
            t_rel = (t_w2b - R_rel @ t_w2t).reshape(3)
            baseline = float(np.linalg.norm(t_rel))
            def euler_deg(R):
                M = np.eye(4); M[:3,:3] = R
                r,p,y = tft.euler_from_matrix(M)
                return (math.degrees(r), math.degrees(p), math.degrees(y))
            self.log.info("TF snapshot:")
            self.log.info(f"  • baseline |t_rel| (top→bottom) = {baseline:.4f} m")
            self.log.info(f"  • top R_euler(deg)={euler_deg(R_w2t)}, t={t_w2t.ravel()}")
            self.log.info(f"  • bot R_euler(deg)={euler_deg(R_w2b)}, t={t_w2b.ravel()}")
        except Exception as e:
            self.log.warning(f"TF snapshot failed: {e}")

    def _reset_samples(self):
        self.samples_obj_eef.clear()
        self.samples_uv_top.clear()
        self.samples_uv_bot.clear()
        self.K_bot = None
        self.D_bot = None
        self.R_bot = None
        self.t_bot = None
        self.state = 'WAIT_TOP'
        self.freeze = False

def main():
    rclpy.init()
    node = BottomCamParamTest()
    try:
        rclpy.spin(node)
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if node is not None and node.bot_cap is not None:
            try: node.bot_cap.release()
            except: pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
