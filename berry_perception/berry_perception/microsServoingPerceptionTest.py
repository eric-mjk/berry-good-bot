#!/usr/bin/env python3
import os, sys, time
import numpy as np
import cv2
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

from .stem_and_yolo import (
    choose_track, segment_strawberry, select_yolo_by_roi_similarity,
    paint_polygons_with_hsv, get_bottom_mask_polygons,
    BOTTOM_MASK_ENABLE, BOTTOM_MASK_HSV, BOTTOM_MASK_FEATHER
)

class IBVSPerceptionTest(Node):
    def __init__(self):
        super().__init__(
            "ibvs_perception_test",
            automatically_declare_parameters_from_overrides=True
        )
        self.log = self.get_logger()
        self.bridge = CvBridge()

        # ───── Params ───────────────────────────────
        self.declare_parameter('top_rgb_topic',   '/camera/camera/color/image_raw')
        self.declare_parameter('top_depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('top_info_topic',  '/camera/camera/color/camera_info')

        self.declare_parameter('top_infer_hz', 3.0)                 # 상단 YOLO 추론 주기(Hz)
        self.declare_parameter('top_grip_roi_xywh', [498,250, 180, 228])  # 상단 gripper ROI (x,y,w,h)
        self.declare_parameter('bottom_infer_hz', 3.0)   # 하단 YOLO 추론 주기(Hz)
        self.declare_parameter('bottom_grip_roi_xywh', [63, 83, 360, 471])  # 하단 gripper ROI (x,y,w,h)

        # 하단 카메라: 토픽 또는 VideoCapture 선택
        self.declare_parameter('bottom_use_v4l', True)      # True: OpenCV VideoCapture
        self.declare_parameter('bottom_v4l_index', 0)       # /dev/video*
        self.declare_parameter('bottom_rgb_topic', '/bottom_cam/image_raw')

        # YOLO
        self.declare_parameter('yolo_model_rel', 'model/model3.pt')
        self.declare_parameter('yolo_conf', 0.01)
        self.declare_parameter('yolo_sim_conf_thresh', 0.30)  # (sim + conf) 유효성 임계값

        # Stem HSV(필요 시 런치/YAML로 오버라이드)
        self.declare_parameter('stem_hsv_low_brown', [10,40,25])
        self.declare_parameter('stem_hsv_high_brown',[30,255,255])
        self.declare_parameter('stem_hsv_low_green', [35,40,25])
        self.declare_parameter('stem_hsv_high_green',[95,255,255])

        # ───── Subscribers ─────────────────────────
        self.top_rgb  = None
        self.top_info = None
        self.sub_rgb  = self.create_subscription(Image, self.get_parameter('top_rgb_topic').value, self.cb_top_rgb,  10)
        self.sub_dep  = self.create_subscription(Image, self.get_parameter('top_depth_topic').value, self.cb_top_depth, 10)
        self.sub_inf  = self.create_subscription(CameraInfo, self.get_parameter('top_info_topic').value, self.cb_cam_info, 10)
        self.top_depth_np = None

        # 하단 카메라
        self.bot_cap = None
        if bool(self.get_parameter('bottom_use_v4l').value):
            idx = int(self.get_parameter('bottom_v4l_index').value)
            self.bot_cap = cv2.VideoCapture(idx)
            if not self.bot_cap.isOpened():
                self.log.error(f"[bottom] cannot open VideoCapture index {idx}")
                self.bot_cap = None
        else:
            self.sub_bot = self.create_subscription(Image, self.get_parameter('bottom_rgb_topic').value, self.cb_bottom_rgb, 10)
            self.bot_img = None

        # ───── YOLO ────────────────────────────────
        pkg_share  = get_package_share_directory('berry_perception')
        model_path = os.path.join(pkg_share, self.get_parameter('yolo_model_rel').value)
        self.log.info(f"[YOLO] loading: {model_path}")
        self.yolo = YOLO(model_path)
        self.yolo.model.to('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.yolo_conf = float(self.get_parameter('yolo_conf').value)
        self.sim_conf_thresh = float(self.get_parameter('yolo_sim_conf_thresh').value)
        # 클래스 이름 매핑
        self.cls_names = self._get_class_names(self.yolo)

        # 하단 추론 주기 제어
        self.bottom_infer_hz = float(self.get_parameter('bottom_infer_hz').value)
        self._bot_last_infer_ts = 0.0
        # 상단 추론 주기 제어
        self.top_infer_hz = float(self.get_parameter('top_infer_hz').value)
        self._top_last_infer_ts = 0.0
        # Gripper ROI 캐시
        self.grip_roi_xywh = [int(v) for v in self.get_parameter('bottom_grip_roi_xywh').value]
        self.top_grip_roi_xywh = [int(v) for v in self.get_parameter('top_grip_roi_xywh').value]
        self._brown_kernel_radius = 5  # 줄기-딸기 접점 검색용 주변 반경(픽셀)

        # 트래킹 상태
        self.prev_bbox_bot  = None
        self._top_last_overlay = None
        self._bot_last_overlay = None  # 최근 추론 결과 오버레이 보존

        # 하단 집게 마스킹 폴리곤(Stem&YOLO 설정에서 읽기)
        self._bottom_polys = get_bottom_mask_polygons()

        # 타이머 루프 (디스플레이/하단 처리)
        self.timer = self.create_timer(1.0/15.0, self.tick)

        # OpenCV 창
        cv2.namedWindow("TOP_RGBD_RAW", cv2.WINDOW_NORMAL)
        cv2.namedWindow("TOP_RGBD_YOLO", cv2.WINDOW_NORMAL)
        cv2.namedWindow("BOTTOM_RGB_RAW", cv2.WINDOW_NORMAL)
        cv2.namedWindow("BOTTOM_RGB_YOLO", cv2.WINDOW_NORMAL)
        self.log.info("▶ IBVS Perception Test started. Press 'q' in any window to quit.")

    # ───── Callbacks ─────────────────────────────
    def cb_top_rgb(self, msg: Image):
        self.top_rgb = msg

    def cb_top_depth(self, msg: Image):
        try:
            self.top_depth_np = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32) / 1000.0
        except Exception:
            self.top_depth_np = None

    def cb_cam_info(self, msg: CameraInfo):
        self.top_info = msg

    def cb_bottom_rgb(self, msg: Image):
        self.bot_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # ───── Helper (geom/ROI) ─────────────────────
    @staticmethod
    def _xywh_to_xyxy(x, y, w, h):
        return (x, y, x + w, y + h)

    @staticmethod
    def _rect_center(x1, y1, x2, y2):
        return (0.5 * (x1 + x2), 0.5 * (y1 + y2))

    @staticmethod
    def _rect_intersects(a, b):
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        iw = max(0, min(ax2, bx2) - max(ax1, bx1))
        ih = max(0, min(ay2, by2) - max(ay1, by1))
        return (iw > 0) and (ih > 0)

    @staticmethod
    def _draw_transparent_mask(dst_bgr, mask, color=(0,255,255), alpha=0.35):
        """mask>0 영역에 반투명 컬러 오버레이"""
        if mask is None or mask.size == 0:
            return dst_bgr
        overlay = dst_bgr.copy()
        col_img = np.zeros_like(dst_bgr, np.uint8)
        col_img[:,:] = color
        overlay[mask>0] = col_img[mask>0]
        out = cv2.addWeighted(overlay, alpha, dst_bgr, 1.0 - alpha, 0)
        return out

    @staticmethod
    def _euclid(p, q):
        return float(np.hypot(p[0]-q[0], p[1]-q[1]))

    @staticmethod
    def _get_class_names(yolo):
        names = getattr(yolo, "names", None)
        if names is None:
            names = getattr(getattr(yolo, "model", None), "names", None)
        if names is None:
            names = {}
        if isinstance(names, list):
            names = {i: n for i, n in enumerate(names)}
        return names

    @staticmethod
    def _draw_transparent_polyline(dst_bgr, pts, colors, thickness=2, alpha=0.5):
        """
        pts: [(x,y), ...]  (인덱스 순서대로 선 연결; y0->y1, y1->y2)
        colors: 각 세그먼트 색상 리스트 (BGR)
        """
        if pts is None or len(pts) < 2:
            return dst_bgr
        overlay = dst_bgr.copy()
        for i in range(len(pts) - 1):
            p1 = (int(pts[i][0]),   int(pts[i][1]))
            p2 = (int(pts[i+1][0]), int(pts[i+1][1]))
            col = colors[i % len(colors)]
            cv2.line(overlay, p1, p2, col, thickness, cv2.LINE_AA)
        return cv2.addWeighted(overlay, alpha, dst_bgr, 1.0 - alpha, 0)

    # ───── Main timer loop ───────────────────────
    def tick(self):
        # 1) 상단 RGB-D
        top_raw = None     # 원본 프레임
        top_dbg = None     # 퍼셉션 오버레이
        if self.top_rgb is not None:
            bgr = self.bridge.imgmsg_to_cv2(self.top_rgb, 'bgr8')
            top_raw = bgr.copy()
            now_t = time.time()
            do_infer_t = True
            if self.top_infer_hz > 0:
                do_infer_t = (now_t - self._top_last_infer_ts) >= (1.0 / self.top_infer_hz)
            if do_infer_t:
                # ── 상단: YOLO + ROI 필터 + Segmentation + Keypoints
                results_t = self.yolo.predict(source=bgr, conf=self.yolo_conf, save=False, verbose=False, max_det=10)
                # xyxy_t, conf_t, kpts_t = self._get_boxes_and_kpts(results_t)
                xyxy_t, conf_t, kpts_t, cls_t = self._get_boxes_and_kpts(results_t)
                # ROI (상단)
                tgx, tgy, tgw, tgh = self.top_grip_roi_xywh
                tgr_xyxy = self._xywh_to_xyxy(tgx, tgy, tgw, tgh)
                cv2.rectangle(bgr, (tgx,tgy), (tgx+tgw, tgy+tgh), (255,255,255), 1)
                # cv2.putText(bgr, "GRIP ROI", (tgx, max(0,tgy-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                cv2.putText(bgr, "GRIP ROI", (tgx, max(0,tgy-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1)

                # ── 선택 로직: 세그먼트 기반 ROI 유사도 최대 박스 선택
                # selected_idx_t, sim_iou_t, seg_roi_t, seg_t = select_yolo_by_roi_similarity(
                    
                selected_idx_t, sim_iou_t, seg_roi_t, seg_t, score_t = select_yolo_by_roi_similarity(
                    bgr=bgr,
                    gr_xyxy=tgr_xyxy,
                    xyxy_list=xyxy_t,
                    conf_list=conf_t,
                    score_thresh=self.sim_conf_thresh
                )
                yolo_valid_t = (selected_idx_t is not None)
                # ── 시각화: 세그먼트 마스크 & 세그 ROI
                mask_t = seg_t.get('mask', None)
                contour_t = seg_t.get('contour', None)
                vis_t = bgr
                if mask_t is not None:
                    vis_t = self._draw_transparent_mask(vis_t, mask_t, color=(0,255,255), alpha=0.35)
                if contour_t is not None and len(contour_t) >= 3:
                    cv2.polylines(vis_t, [contour_t.reshape(-1,1,2)], isClosed=True, color=(0,128,255), thickness=2)
                    cv2.putText(vis_t, "SEG", (int(contour_t[0,0]), max(0, int(contour_t[0,1]-8))),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,128,255), 1)
                    
                # 세그 ROI (mask로 만든 ROI) 시각화
                if seg_roi_t is not None:
                    sx1,sy1,sx2,sy2 = seg_roi_t
                    cv2.rectangle(vis_t, (sx1,sy1), (sx2,sy2), (0,128,255), 1)
                    cv2.putText(vis_t, "SEG ROI", (sx1, max(0, sy1-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,128,255), 1)
                # 선택된 YOLO bbox 시각화 + conf & sim
                if yolo_valid_t:
                    tx1,ty1,tx2,ty2 = xyxy_t[selected_idx_t]
                    cv2.rectangle(vis_t, (tx1,ty1), (tx2,ty2), (0,255,0), 2)
                    tcx_i, tcy_i = int((tx1+tx2)/2), int((ty1+ty2)/2)
                    cv2.circle(vis_t, (tcx_i,tcy_i), 4, (0,0,255), -1)
                    # 작은 글씨로 conf와 ROI 유사도(iou) 함께 출력
                    conf_val = conf_t[selected_idx_t] if (conf_t and selected_idx_t < len(conf_t)) else 0.0
                    # cv2.putText(vis_t, f"conf={conf_val:.2f}, sim={sim_iou_t:.2f}", (tx1, max(0, ty1-22)),

                    # cv2.putText(vis_t, f"conf={conf_val:.2f}, sim={sim_iou_t:.2f}, score={(conf_val+sim_iou_t):.2f}", (tx1, max(0, ty1-22)),                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)
                    # cv2.putText(vis_t, f"YOLO OK ({tcx_i},{tcy_i})", (tx1, max(0, ty1-6)),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    cv2.putText(vis_t, f"conf={conf_val:.2f}, sim={sim_iou_t:.2f}, score={(conf_val+sim_iou_t):.2f}", (tx1, max(0, ty1-22)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)
                    cv2.putText(vis_t, f"YOLO OK ({tcx_i},{tcy_i})", (tx1, max(0, ty1-6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 2)
                    # ── 클래스 라벨(작은 글씨, 배경 박스)
                    cls_id_t = int(cls_t[selected_idx_t]) if (cls_t and selected_idx_t < len(cls_t)) else -1
                    cls_name_t = self.cls_names.get(cls_id_t, str(cls_id_t))
                    label_t = f"{cls_name_t} {conf_val:.2f}"
                    (tw, th), _ = cv2.getTextSize(label_t, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                    cv2.rectangle(vis_t, (tx1, ty1 - th - 6), (tx1 + tw + 2, ty1), (0,255,0), -1)
                    cv2.putText(vis_t, label_t, (tx1 + 1, ty1 - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,0,0), 1)
                else:
                    # 유효하지 않을 때 임계값/점수 표시
                    cv2.putText(vis_t, f"YOLO not valid (score={score_t:.2f} <= thr={self.sim_conf_thresh:.2f}) -> use GRIP ROI",
                                (max(0, tgx-10), max(0, tgy-22)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

                # YOLO keypoints (상단) - 서로 다른 색
                if yolo_valid_t and kpts_t and selected_idx_t < len(kpts_t):
                    # palette = [(255,0,255), (255,255,0), (0,165,255)]  # magenta, cyan, orange (BGR)
                    # for j,(kx,ky) in enumerate(kpts_t[selected_idx_t][:3]):
                    #     cv2.circle(vis_t, (int(kx), int(ky)), 4, palette[j%3], -1)
                    #     cv2.putText(vis_t, f"y{j}", (int(kx)+3, int(ky)-3),
                    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.45, palette[j%3], 1)
                    palette = [(255,0,255), (255,255,0), (0,165,255)]  # magenta, cyan, orange (BGR)
                    kp_pts_t = []
                    for j,(kx,ky) in enumerate(kpts_t[selected_idx_t][:3]):
                        cv2.circle(vis_t, (int(kx), int(ky)), 4, palette[j%3], -1)
                        cv2.putText(vis_t, f"y{j}", (int(kx)+3, int(ky)-3),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, palette[j%3], 1)
                        kp_pts_t.append((kx,ky))
                    # y0->y1->y2 반투명 선
                    if len(kp_pts_t) >= 2:
                        vis_t = self._draw_transparent_polyline(vis_t, kp_pts_t, palette, thickness=2, alpha=0.5)

                self._top_last_overlay = vis_t.copy()
                top_dbg = vis_t
                self._top_last_infer_ts = now_t
            else:
                top_dbg = self._top_last_overlay if self._top_last_overlay is not None else top_raw
  
        # 2) 하단 RGB: YOLO bbox/center (VideoCapture 또는 토픽)
        bot_dbg = None
        bot_raw = None     # 원본 프레임
        frame = None
        brown_lo = tuple(self.get_parameter('stem_hsv_low_brown').value)
        brown_hi = tuple(self.get_parameter('stem_hsv_high_brown').value)
        if self.bot_cap is not None:
            ok, frame = self.bot_cap.read()
            if not ok: frame = None
        else:
            frame = getattr(self, 'bot_img', None)

        if frame is not None:
            bot_raw = frame.copy()  # 원본 보존

            # # === (추가) 집게 가리기 전처리: stem_and_yolo.py 설정 사용 ===
            # if BOTTOM_MASK_ENABLE and len(self._bottom_polys) > 0:
            #     frame, _ = paint_polygons_with_hsv(
            #         frame,
            #         self._bottom_polys,
            #         hsv_color=BOTTOM_MASK_HSV,
            #         feather=int(BOTTOM_MASK_FEATHER),
            #         alpha=1.0
            #     )
            # ============================================================

            now = time.time()
            do_infer = True
            if self.bottom_infer_hz > 0:
                do_infer = (now - self._bot_last_infer_ts) >= (1.0 / self.bottom_infer_hz)
            if do_infer:
                results_b = self.yolo.predict(source=frame, conf=self.yolo_conf, save=False, verbose=False, max_det=10)
                # xyxy_b, conf_b, kpts_b = self._get_boxes_and_kpts(results_b)
                xyxy_b, conf_b, kpts_b, cls_b = self._get_boxes_and_kpts(results_b)

                # ── Gripper ROI
                gx, gy, gw, gh = self.grip_roi_xywh
                gr_xyxy = self._xywh_to_xyxy(gx, gy, gw, gh)
                cv2.rectangle(frame, (gx,gy), (gx+gw, gy+gh), (255,255,255), 1)
                # cv2.putText(frame, "GRIP ROI", (gx, max(0,gy-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                cv2.putText(frame, "GRIP ROI", (gx, max(0,gy-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1)

                # # ── YOLO 유효성 필터: ROI와 겹치는 박스만
                selected_idx, sim_iou, seg_roi, seg, score = select_yolo_by_roi_similarity(
                    bgr=frame,
                    gr_xyxy=gr_xyxy,
                    xyxy_list=xyxy_b,
                    conf_list=conf_b,
                    score_thresh=self.sim_conf_thresh
                )
                yolo_valid = (selected_idx is not None)
                mask = seg.get('mask', None)
                contour = seg.get('contour', None)

                # ── Segmentation 시각화: 반투명 마스크 + 컨투어
                vis = frame
                if mask is not None:
                    vis = self._draw_transparent_mask(vis, mask, color=(0,255,255), alpha=0.35)  # 노란-청록 톤
                if contour is not None and len(contour) >= 3:
                    cv2.polylines(vis, [contour.reshape(-1,1,2)], isClosed=True, color=(0,128,255), thickness=2)
                    cv2.putText(vis, "SEG", (int(contour[0,0]), max(0, int(contour[0,1]-8))),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,128,255), 1)
                    
                # 세그 ROI (mask로 만든 ROI) 시각화
                if seg_roi is not None:
                    sx1,sy1,sx2,sy2 = seg_roi
                    cv2.rectangle(vis, (sx1,sy1), (sx2,sy2), (0,128,255), 1)
                    cv2.putText(vis, "SEG ROI", (sx1, max(0, sy1-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,128,255), 1)

                # ── YOLO bbox/center 선택 결과 시각화 + conf & sim
                if yolo_valid:
                    x1,y1,x2,y2 = xyxy_b[selected_idx]
                    self.prev_bbox_bot = [x1,y1,x2,y2]
                    cv2.rectangle(vis, (x1,y1), (x2,y2), (0,255,0), 2)
                    cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                    cv2.circle(vis, (cx,cy), 4, (0,0,255), -1)
                    conf_val = conf_b[selected_idx] if (conf_b and selected_idx < len(conf_b)) else 0.0
                    
                    cv2.putText(vis, f"conf={conf_val:.2f}, sim={sim_iou:.2f}, score={(conf_val+sim_iou):.2f}", (x1, max(0, y1-22)),                                
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)
                    
                    cv2.putText(vis, f"YOLO OK  ({cx},{cy})", (x1, max(0, y1-6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 2)
                    # ── 클래스 라벨(작은 글씨, 배경 박스)
                    cls_id_b = int(cls_b[selected_idx]) if (cls_b and selected_idx < len(cls_b)) else -1
                    cls_name_b = self.cls_names.get(cls_id_b, str(cls_id_b))
                    label_b = f"{cls_name_b} {conf_val:.2f}"
                    (tw, th), _ = cv2.getTextSize(label_b, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                    cv2.rectangle(vis, (x1, y1 - th - 6), (x1 + tw + 2, y1), (0,255,0), -1)
                    cv2.putText(vis, label_b, (x1 + 1, y1 - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,0,0), 1)
                else:
                    self.prev_bbox_bot = None
                    # cv2.putText(vis, "YOLO not valid → use GRIP ROI", (gx, max(0,gy-22)),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)
                    cv2.putText(vis, f"YOLO not valid (score={score:.2f} <= thr={self.sim_conf_thresh:.2f}) -> use GRIP ROI",
                                (gx, max(0,gy-22)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

                # ── YOLO keypoints 시각화 (3개 각기 다른 색)
                if yolo_valid and kpts_b and selected_idx < len(kpts_b):
                    # palette = [(255,0,255), (255,255,0), (0,165,255)]  # magenta, cyan, orange (BGR)
                    # for j,(kx,ky) in enumerate(kpts_b[selected_idx][:3]):  # 최대 3개만
                    #     cv2.circle(vis, (int(kx), int(ky)), 4, palette[j%3], -1)
                    #     cv2.putText(vis, f"y{j}", (int(kx)+3, int(ky)-3),
                    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.45, palette[j%3], 1)
                    palette = [(255,0,255), (255,255,0), (0,165,255)]  # magenta, cyan, orange (BGR)
                    kp_pts_b = []
                    for j,(kx,ky) in enumerate(kpts_b[selected_idx][:3]):  # 최대 3개만
                        cv2.circle(vis, (int(kx), int(ky)), 4, palette[j%3], -1)
                        cv2.putText(vis, f"y{j}", (int(kx)+3, int(ky)-3),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, palette[j%3], 1)
                        kp_pts_b.append((kx,ky))
                    # y0->y1->y2 반투명 선
                    if len(kp_pts_b) >= 2:
                        vis = self._draw_transparent_polyline(vis, kp_pts_b, palette, thickness=2, alpha=0.5)

                # 결과 반영
                self._bot_last_infer_ts = now
                # self._bot_last_overlay = frame.copy()
                # bot_dbg = frame
                self._bot_last_overlay = vis.copy()
                bot_dbg = vis
            else:
                # 추론 주기 사이에는 마지막 오버레이 또는 원본 프레임을 표시
                bot_dbg = self._bot_last_overlay if self._bot_last_overlay is not None else frame
                bot_raw = frame if bot_raw is None else bot_raw

        # 3) 디스플레이
        # 상단
        if top_raw is not None:
            cv2.imshow("TOP_RGBD_RAW", top_raw)
        if top_dbg is not None:
            cv2.imshow("TOP_RGBD_YOLO", top_dbg)
        # 하단
        if bot_raw is not None:
            cv2.imshow("BOTTOM_RGB_RAW", bot_raw)
        if bot_dbg is not None:
             cv2.imshow("BOTTOM_RGB_YOLO", bot_dbg)

        # 종료 키
        if (top_raw is not None) or (top_dbg is not None) or (bot_dbg is not None):
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.log.info("⏹ Quit requested (q)")
                rclpy.shutdown()

    # ───── YOLO util ─────────────────────────────
    def _get_boxes_and_kpts(self, results):
        """
        YOLO 결과로부터 bbox, conf, keypoints를 추출.
        반환:
          xyxy: List[List[int]]
          conf: List[float]
          kpts: List[List[Tuple[float, float]]]  # 각 인스턴스별 (x,y) 리스트, keypoints가 없으면 빈 리스트
          clses: List[int]  # 클래스 id
        """
        if (not results) or (results[0].boxes is None) or (len(results[0].boxes) == 0):
            return [], [], [], []
        b = results[0].boxes
        # bbox / conf
        try:
            xyxy = b.xyxy.cpu().numpy()
            conf = b.conf.cpu().numpy()
        except Exception:
            xyxy = b.xyxy.numpy() if hasattr(b.xyxy, "numpy") else np.array(b.xyxy)
            conf = b.conf.numpy() if hasattr(b.conf, "numpy") else np.array(b.conf)
        xyxy = xyxy.astype(int).tolist()
        conf = [float(c) for c in conf.tolist()]
        # classes
        try:
            cls = b.cls.detach().cpu().numpy().astype(int)
        except Exception:
            cls = np.array([]).astype(int)
        clses = cls.tolist() if cls.size > 0 else [ -1 for _ in xyxy ]
        # keypoints (pose 모델일 경우)
        kpts = []
        kobj = getattr(results[0], "keypoints", None)
        if kobj is not None:
            try:
                karr = kobj.xy
                karr = karr.detach().cpu().numpy() if hasattr(karr, "detach") else np.array(karr)
                # karr shape: [N, K, 2]
                for inst in karr:
                    kpts.append([(float(p[0]), float(p[1])) for p in inst])
            except Exception:
                # keypoints가 기대와 다를 경우 안전하게 빈 리스트로 처리
                kpts = [[] for _ in xyxy]
        else:
            kpts = [[] for _ in xyxy]
        return xyxy, conf, kpts, clses

def main():
    rclpy.init()
    node = IBVSPerceptionTest()
    try:
        rclpy.spin(node)
    finally:
        if node.bot_cap is not None:
            try: node.bot_cap.release()
            except: pass
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
