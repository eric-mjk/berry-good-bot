#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DatasetCapture2Hz
 - 하단 RGB(USB/V4L 또는 ROS 토픽) + 상단 RGBD(ROS) 프레임을 2 Hz로 저장
 - YOLO 학습용: full 프레임 + ROI crop(파라미터 기반) 동시 저장 옵션

[저장 구조]
 out_root/
   bottom_full/
   bottom_roi/
   top_full/
   top_roi/
   top_depth/   (옵션: 16-bit PNG, 단위=mm)

[중요 파라미터]
 - use_v4l (bool, default=True): 하단 카메라를 VideoCapture로 읽기
 - v4l_index (int, default=0): VideoCapture 인덱스
 - bottom_rgb_topic (str): use_v4l=False인 경우 구독할 토픽
 - top_rgb_topic, top_depth_topic (str): 상단 RGB/Depth 토픽
 - grip_roi_xywh (int[4]): 하단 ROI (x,y,w,h)
 - top_grip_roi_xywh (int[4]): 상단 ROI (x,y,w,h)
 - out_root (str): 저장 루트 경로 (기본: ~/ibvs_dataset)
 - hz (float, default=2.0): 저장 주기(Hz)
 - save_full (bool, default=True), save_roi (bool, default=True), save_depth (bool, default=True)
 - image_ext ("jpg"|"png", default="jpg"): RGB 저장 포맷
 - jpeg_quality (0~100, default=95), png_compress (0~9, default=3)

[실행]
  ros2 run <your_pkg> dataset_capture_2hz  \
    --ros-args \
      -p use_v4l:=true -p v4l_index:=0 \
      -p top_rgb_topic:=/camera/camera/color/image_raw \
      -p top_depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
      -p bottom_rgb_topic:=/bottom_cam/image_raw \
      -p out_root:="/data/yolo_ds" \
      -p grip_roi_xywh:="[63,83,360,471]" \
      -p top_grip_roi_xywh:="[498,250,310,228]" \
      -p hz:=2.0 -p save_depth:=true

메모:
 - depth 입력 형식이 32FC1(미터)면 mm 변환 후 uint16 PNG로 저장, 16UC1(밀리미터)이면 그대로 저장합니다.
 - ROI가 프레임 바깥을 넘으면 자동으로 클리핑합니다.
"""
import os
import time
import math
import pathlib
import threading
from typing import Optional, Tuple, List

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DatasetCapture2Hz(Node):
    def __init__(self):
        super().__init__(
            "dataset_capture_2hz",
            automatically_declare_parameters_from_overrides=True
        )
        self.log = self.get_logger()
        self.bridge = CvBridge()
        self.cb_grp = ReentrantCallbackGroup()

        # ───── 파라미터 ─────
        self.declare_parameter("use_v4l", True)
        self.declare_parameter("v4l_index", 0)
        self.declare_parameter("bottom_rgb_topic", "/bottom_cam/image_raw")

        self.declare_parameter("top_rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("top_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")

        self.declare_parameter("grip_roi_xywh", [63, 83, 360, 471])
        self.declare_parameter("top_grip_roi_xywh", [498, 250, 310, 228])

        self.declare_parameter("out_root", os.path.expanduser("~/ibvs_dataset"))
        self.declare_parameter("hz", 2.0)
        self.declare_parameter("save_full", True)
        self.declare_parameter("save_roi", False)
        self.declare_parameter("save_depth", False)
        self.declare_parameter("image_ext", "jpg")  # jpg|png
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("png_compress", 3)
        self.declare_parameter("stamp_from_top_header", True)

        # 내부 상태
        self._use_v4l = bool(self.get_parameter("use_v4l").value)
        self._v4l_index = int(self.get_parameter("v4l_index").value)
        self._bottom_topic = str(self.get_parameter("bottom_rgb_topic").value)
        self._top_rgb_topic = str(self.get_parameter("top_rgb_topic").value)
        self._top_depth_topic = str(self.get_parameter("top_depth_topic").value)

        self._grip_roi = [int(v) for v in self.get_parameter("grip_roi_xywh").value]
        self._top_roi = [int(v) for v in self.get_parameter("top_grip_roi_xywh").value]

        self._out_root = str(self.get_parameter("out_root").value)
        self._hz = float(self.get_parameter("hz").value)
        self._save_full = bool(self.get_parameter("save_full").value)
        self._save_roi = bool(self.get_parameter("save_roi").value)
        self._save_depth = bool(self.get_parameter("save_depth").value)
        self._ext = str(self.get_parameter("image_ext").value).lower()
        self._jpg_q = int(self.get_parameter("jpeg_quality").value)
        self._png_c = int(self.get_parameter("png_compress").value)
        self._stamp_from_top_hdr = bool(self.get_parameter("stamp_from_top_header").value)

        if self._ext not in ("jpg", "jpeg", "png"):
            self.log.warn(f"Unsupported image_ext={self._ext}, fallback to 'jpg'")
            self._ext = "jpg"

        # 저장 디렉토리 준비
        self.dir_bottom_full = pathlib.Path(self._out_root) / "bottom_full"
        self.dir_bottom_roi  = pathlib.Path(self._out_root) / "bottom_roi"
        self.dir_top_full    = pathlib.Path(self._out_root) / "top_full"
        self.dir_top_roi     = pathlib.Path(self._out_root) / "top_roi"
        self.dir_top_depth   = pathlib.Path(self._out_root) / "top_depth"
        for d in [self.dir_bottom_full, self.dir_bottom_roi, self.dir_top_full, self.dir_top_roi, self.dir_top_depth]:
            d.mkdir(parents=True, exist_ok=True)

        # 프레임 버퍼
        self._bot_bgr: Optional[np.ndarray] = None
        self._top_bgr: Optional[np.ndarray] = None
        self._top_depth: Optional[np.ndarray] = None  # float m or uint16 mm
        self._top_last_stamp_ns: Optional[int] = None
        self._lock = threading.Lock()

        # 입력 소스
        self.cap = None
        if self._use_v4l:
            self.cap = cv2.VideoCapture(self._v4l_index)
            if not self.cap.isOpened():
                self.log.error(f"[camera] cannot open VideoCapture index {self._v4l_index}")
                self.cap = None
            else:
                self.log.info(f"[camera] VideoCapture index {self._v4l_index} opened")
        else:
            self.create_subscription(Image, self._bottom_topic, self._cb_bottom_rgb, 10)

        # 상단 RGBD 구독
        self.create_subscription(Image, self._top_rgb_topic, self._cb_top_rgb, 10)
        self.create_subscription(Image, self._top_depth_topic, self._cb_top_depth, 10)

        # 타이머
        dt = 1.0 / max(1e-3, self._hz)
        self.timer = self.create_timer(dt, self._on_tick, callback_group=self.cb_grp)

        self._counter = 0
        self.log.info(f"▶ DatasetCapture2Hz started: saving to '{self._out_root}' @ {self._hz:.2f} Hz")

    # ──────────────────────────
    # Callbacks
    # ──────────────────────────
    def _cb_bottom_rgb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock:
            self._bot_bgr = bgr

    def _cb_top_rgb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock:
            self._top_bgr = bgr
            if self._stamp_from_top_hdr:
                self._top_last_stamp_ns = (int(msg.header.stamp.sec) * 1_000_000_000) + int(msg.header.stamp.nanosec)

    def _cb_top_depth(self, msg: Image):
        # 가능한 포맷: 16UC1(mm) 또는 32FC1(m)
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        with self._lock:
            self._top_depth = arr

    # ──────────────────────────
    # Helpers
    # ──────────────────────────
    @staticmethod
    def _clip_roi(x: int, y: int, w: int, h: int, width: int, height: int) -> Tuple[int, int, int, int]:
        x = max(0, x); y = max(0, y)
        w = max(0, min(w, width - x))
        h = max(0, min(h, height - y))
        return x, y, w, h

    def _save_image(self, path: pathlib.Path, img: np.ndarray) -> bool:
        params: List[int] = []
        if self._ext in ("jpg", "jpeg"):
            params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self._jpg_q)]
        elif self._ext == "png":
            params = [int(cv2.IMWRITE_PNG_COMPRESSION), int(self._png_c)]
        return bool(cv2.imwrite(str(path), img, params))

    def _save_depth_png16(self, path: pathlib.Path, depth: np.ndarray) -> bool:
        # 허용 입력: float32(미터) 또는 uint16(밀리미터)
        if depth is None:
            return False
        if depth.dtype == np.float32 or depth.dtype == np.float64:
            mm = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
            mm = np.clip(mm, 0.0, 65535.0).astype(np.uint16)
        elif depth.dtype == np.uint16:
            mm = depth
        else:
            # 기타 타입은 시도 변환
            try:
                mm = np.nan_to_num(depth.astype(np.float32), nan=0.0) * 1000.0
                mm = np.clip(mm, 0.0, 65535.0).astype(np.uint16)
            except Exception:
                self.log.warn(f"[depth] unsupported dtype {depth.dtype}, skip")
                return False
        return bool(cv2.imwrite(str(path), mm))  # 16-bit PNG

    def _now_stamp_ns(self) -> int:
        if self._stamp_from_top_hdr and self._top_last_stamp_ns is not None:
            return int(self._top_last_stamp_ns)
        # wall clock fallback
        return int(time.time_ns())

    # ──────────────────────────
    # Timer tick: capture & save
    # ──────────────────────────
    def _on_tick(self):
        # 1) 최신 프레임 스냅샷 (thread-safe)
        with self._lock:
            top_bgr = None if self._top_bgr is None else self._top_bgr.copy()
            top_depth = None if self._top_depth is None else self._top_depth.copy()
            bot_bgr_cb = None if self._bot_bgr is None else self._bot_bgr.copy()

        # V4L이면 지금 프레임 읽기
        bot_bgr = bot_bgr_cb
        if self.cap is not None:
            ok, frm = self.cap.read()
            if ok and frm is not None:
                bot_bgr = frm

        # 아무 것도 없으면 패스
        if bot_bgr is None and top_bgr is None:
            return

        ts_ns = self._now_stamp_ns()
        base = f"{ts_ns}"
        ext = self._ext

        # 2) 하단 저장
        if bot_bgr is not None:
            h, w = bot_bgr.shape[:2]
            if self._save_full:
                p = self.dir_bottom_full / f"bot_{base}.{ext}"
                self._save_image(p, bot_bgr)
            if self._save_roi:
                x, y, rw, rh = self._clip_roi(*self._grip_roi, w, h)
                if rw > 0 and rh > 0:
                    crop = bot_bgr[y:y+rh, x:x+rw].copy()
                    p = self.dir_bottom_roi / f"bot_{base}_roi.{ext}"
                    self._save_image(p, crop)

        # 3) 상단 저장 (RGB/Depth)
        if top_bgr is not None:
            th, tw = top_bgr.shape[:2]
            if self._save_full:
                p = self.dir_top_full / f"top_{base}.{ext}"
                self._save_image(p, top_bgr)
            if self._save_roi:
                x, y, rw, rh = self._clip_roi(*self._top_roi, tw, th)
                if rw > 0 and rh > 0:
                    crop = top_bgr[y:y+rh, x:x+rw].copy()
                    p = self.dir_top_roi / f"top_{base}_roi.{ext}"
                    self._save_image(p, crop)

        if self._save_depth and top_depth is not None:
            p = self.dir_top_depth / f"top_{base}_depth.png"  # 고정: 16-bit PNG
            self._save_depth_png16(p, top_depth)

        self._counter += 1
        if self._counter % 20 == 0:
            self.log.info(f"saved {self._counter} ticks (≈{self._counter/self._hz:.1f}s)")


def main():
    rclpy.init()
    node = DatasetCapture2Hz()
    try:
        rclpy.spin(node)
    finally:
        if node.cap is not None:
            try:
                node.cap.release()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
