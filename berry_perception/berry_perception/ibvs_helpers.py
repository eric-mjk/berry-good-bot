#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ibvs_helpers
 - visualServoing.py에서 공통/수학/시각화/TF 관련 유틸을 분리
 - 모든 함수는 side-effect 최소화. logger, pub 등 외부 객체는 인자로 전달.
"""
from typing import List, Tuple, Optional
import math, numpy as np, cv2, rclpy
import tf_transformations as tft
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf2_ros import Buffer

# ──────────────────────────────────────────────────────────────
# 고정 좌표 변환 (optical → body(camera_link))
# OpenCV optical: x=우, y=하, z=전방
# body(camera_link): x=전방, y=좌, z=상
# body = R_LINK_FROM_OPT @ optical
R_LINK_FROM_OPT = np.array([[ 0,  0,  1],
                            [-1,  0,  0],
                            [ 0, -1,  0]], dtype=np.float64)

# ──────────────────────────────────────────────────────────────
# 순수 수학/도우미
# ──────────────────────────────────────────────────────────────
def xywh_to_xyxy(x: float, y: float, w: float, h: float) -> Tuple[int,int,int,int]:
    return (int(x), int(y), int(x + w), int(y + h))

def saturate(x: float, lim: float) -> float:
    if x >  lim: return  lim
    if x < -lim: return -lim
    return x

def quat_to_R(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    return tft.quaternion_matrix((qx, qy, qz, qw))[:3, :3]

def boxes_conf_kpts(results) -> Tuple[List[List[int]], List[float], List[List[Tuple[float,float]]]]:
    """
    YOLO results → (xyxy, conf, keypoints)
    xyxy: List[[x1,y1,x2,y2]] (int), conf: List[float],
    kpts: List[List[(x,y)]], 없으면 빈 리스트
    """
    if (not results) or (results[0].boxes is None) or (len(results[0].boxes) == 0):
        return [], [], []
    b = results[0].boxes
    try:
        xyxy = b.xyxy.detach().cpu().numpy(); conf = b.conf.detach().cpu().numpy()
    except Exception:
        xyxy = b.xyxy.numpy(); conf = b.conf.numpy()
    xyxy = xyxy.astype(int).tolist()
    conf = [float(c) for c in conf.tolist()]
    # keypoints
    kpts = []
    kobj = getattr(results[0], "keypoints", None)
    if kobj is not None and kobj.xy is not None:
        try:
            karr = kobj.xy.detach().cpu().numpy()  # [N,K,2]
        except Exception:
            karr = kobj.xy
        for inst in karr:
            kpts.append([(float(p[0]), float(p[1])) for p in inst])
    else:
        kpts = [[] for _ in xyxy]
    return xyxy, conf, kpts

def pixel_to_ray_opt(u: float, v: float, fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    """픽셀 → optical frame 정규화 레이 (단위벡터, z>0)"""
    x = (u - cx) / max(1e-9, fx)
    y = (v - cy) / max(1e-9, fy)
    r = np.array([x, y, 1.0], dtype=np.float64)
    return r / np.linalg.norm(r)

def point_on_ray1_closest_to_ray2(p1: np.ndarray, d1: np.ndarray,
                                  p2: np.ndarray, d2: np.ndarray) -> Tuple[np.ndarray, float, float]:
    """
    p1 + s*d1 (ray1)와 p2 + t*d2 (ray2) 사이 최단거리에서
    **ray1 위의 점(P1)**만 반환 (깊이만 추정).
    반환: (P1, gap, s)
    """
    d1 = d1 / (np.linalg.norm(d1) + 1e-12)
    d2 = d2 / (np.linalg.norm(d2) + 1e-12)
    # 표준 유도에 맞춰 w0 = p1 - p2 를 사용해야 s, t 부호가 올바릅니다.
    w0 = p1 - p2
    a = float(np.dot(d1, d1)); b = float(np.dot(d1, d2)); c = float(np.dot(d2, d2))
    d = float(np.dot(d1, w0)); e = float(np.dot(d2, w0))
    denom = (a*c - b*b)
    if abs(denom) < 1e-12:
        # 거의 평행: ray1에서 s=0으로 두고 ray2 쪽 파라미터만 계산
        s = 0.0
        t = e / max(1e-12, b)
    else:
        s = (b*e - c*d) / denom
        t = (a*e - b*d) / denom
    
    P1 = p1 + s * d1
    P2 = p2 + t * d2
    gap = float(np.linalg.norm(P1 - P2))
    return P1, gap, float(s)

def draw_kpts(vis: np.ndarray, kpts_xy: list, color=(0,255,255), rad=4, thick=2, idx_text=False):
    if vis is None or kpts_xy is None: return
    for i,xy in enumerate(kpts_xy):
        if xy is None: continue
        u,v = int(round(xy[0])), int(round(xy[1]))
        cv2.circle(vis, (u,v), rad, color, -1)
        if idx_text:
            cv2.putText(vis, f"{i}", (u+4, max(0,v-4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

# ──────────────────────────────────────────────────────────────
# TF / 좌표 변환
# ──────────────────────────────────────────────────────────────
def lookup_Rt_base_from_link(tf_buffer: Buffer, base_frame: str, link_frame: str):
    """base ← link (body) 의 (R,t)"""
    tf = tf_buffer.lookup_transform(base_frame, link_frame, rclpy.time.Time())
    R = quat_to_R(tf.transform.rotation.x, tf.transform.rotation.y,
                  tf.transform.rotation.z, tf.transform.rotation.w)
    t = np.array([tf.transform.translation.x,
                  tf.transform.translation.y,
                  tf.transform.translation.z], dtype=np.float64)
    return R, t

def omega_base_from_omega_cam(tf_buffer: Buffer, base_frame: str, cam_frame: str,
                              w_cam3: np.ndarray) -> np.ndarray:
    """
    w_cam3: [wx, wy, wz] in bottom camera frame (cam_frame).
    returns: [wx, wy, wz] in base_frame.
    """
    tf_base_from_cam = tf_buffer.lookup_transform(base_frame, cam_frame, rclpy.time.Time())
    R_base_from_cam = quat_to_R(tf_base_from_cam.transform.rotation.x,
                                tf_base_from_cam.transform.rotation.y,
                                tf_base_from_cam.transform.rotation.z,
                                tf_base_from_cam.transform.rotation.w)
    return (R_base_from_cam @ w_cam3.reshape(3, 1)).reshape(3)

# ──────────────────────────────────────────────────────────────
# 시각화 / 메시지
# ──────────────────────────────────────────────────────────────
def publish_marker(marker_pub, frame_id: str, stamp, pos_base: np.ndarray,
                   color=(1.0, 0.2, 0.2), ns="ibvs", mid=10, scale=0.02):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns; m.id = mid; m.type = Marker.SPHERE; m.action = Marker.ADD
    m.pose.position.x = float(pos_base[0])
    m.pose.position.y = float(pos_base[1])
    m.pose.position.z = float(pos_base[2])
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 0.9
    marker_pub.publish(m)
