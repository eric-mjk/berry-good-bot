#!/usr/bin/env python3
# yolo_fruit_cam.py
import os
import time
import argparse
from collections import deque

import cv2
import numpy as np
import torch
from ultralytics import YOLO


def parse_args():
    p = argparse.ArgumentParser(
        description="Bottom RGB camera real-time inference (Ultralytics YOLO) without ROS2"
    )
    p.add_argument("--model", type=str, default="../model/model3.pt",
                   help="Path to trained YOLO model (e.g., model/fruit.pt)")
    p.add_argument("--index", type=int, default=0,
                   help="VideoCapture index for bottom RGB camera (default: 0)")
    p.add_argument("--hz", type=float, default=5.0,
                   help="Target inference rate in Hz (0 = every frame)")
    p.add_argument("--conf", type=float, default=0.25,
                   help="Confidence threshold")
    p.add_argument("--max-det", type=int, default=50,
                   help="Max detections per frame")
    p.add_argument("--width", type=int, default=0,
                   help="Optional capture width (0 = keep default)")
    p.add_argument("--height", type=int, default=0,
                   help="Optional capture height (0 = keep default)")
    p.add_argument("--window", type=str, default="BOTTOM_CAM_YOLO",
                   help="CV2 window name")
    return p.parse_args()


def get_names(model):
    # robustly resolve class names across ultralytics versions
    names = getattr(model, "names", None)
    if names is None:
        names = getattr(getattr(model, "model", None), "names", None)
    if names is None:
        names = {}
    if isinstance(names, list):
        names = {i: n for i, n in enumerate(names)}
    return names


def class_color(cls_id: int) -> tuple:
    # Deterministic distinct-ish BGR color per class id using HSV→BGR
    h = int((cls_id * 47) % 180)  # OpenCV hue range: [0,179]
    hsv = np.uint8([[[h, 200, 255]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


# def draw_detections(img, boxes_xyxy, confs, clses, names):
def draw_detections(img, boxes_xyxy, confs, clses, names, kpts=None):
    vis = img.copy()
    # for (x1, y1, x2, y2), conf, cls_id in zip(boxes_xyxy, confs, clses):
    # 작은 폰트/두께로 전반 텍스트 축소
    label_font_scale = 0.45
    label_thickness  = 1
    # keypoint 팔레트(y0,y1,y2 각각 다른 색)
    kp_palette = [(255, 0, 255), (255, 255, 0), (0, 165, 255)]  # BGR: magenta, cyan, orange

    for i, ((x1, y1, x2, y2), conf, cls_id) in enumerate(zip(boxes_xyxy, confs, clses)):
        color = class_color(int(cls_id))
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        # label = f"{names.get(int(cls_id), str(int(cls_id)))} {conf:.2f}"
        # (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        # cv2.rectangle(vis, (x1, y1 - th - 6), (x1 + tw + 2, y1), color, -1)
        # cv2.putText(vis, label, (x1 + 1, y1 - 4),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)
        label = f"{names.get(int(cls_id), str(int(cls_id)))} {conf:.2f}"
        (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, label_font_scale, label_thickness+1)
        cv2.rectangle(vis, (x1, y1 - th - 6), (x1 + tw + 2, y1), color, -1)
        cv2.putText(vis, label, (x1 + 1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, label_font_scale, (0, 0, 0), label_thickness+1)

        # ── Keypoints (y0,y1,y2)
        if kpts is not None and i < len(kpts) and len(kpts[i]) > 0:
            for j, (kx, ky) in enumerate(kpts[i][:3]):  # 최대 3개만 (y0,y1,y2)
                if kx is None or ky is None:
                    continue
                cv2.circle(vis, (int(kx), int(ky)), 3, kp_palette[j % 3], -1)
                # keypoint 이름을 작게 표기
                cv2.putText(vis, f"y{j}", (int(kx) + 3, int(ky) - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, kp_palette[j % 3], 1)
    return vis


def main():
    args = parse_args()

    # ── VideoCapture
    cap = cv2.VideoCapture(args.index)
    if not cap.isOpened():
        raise SystemExit(f"[Error] Cannot open VideoCapture index {args.index}")

    if args.width > 0:  cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0: cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # ── YOLO model
    print(f"[YOLO] loading: {args.model}")
    model = YOLO(args.model)
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    model.model.to(device)
    names = get_names(model)
    print(f"[YOLO] device: {device}, classes: {names}")

    # ── Control for target Hz
    target_hz = max(0.0, float(args.hz))
    min_period = 0.0 if target_hz <= 0 else (1.0 / target_hz)
    last_infer_ts = 0.0
    last_overlay = None

    # Stats
    fps_hist = deque(maxlen=30)
    ihz_hist = deque(maxlen=30)  # actual inference Hz
    last_frame_ts = time.time()

    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    print("[Info] Press 'q' to quit | '+' / '-' to adjust Hz | 'h' to toggle help")
    show_help = True

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[Warn] Failed to read frame.")
            break

        now = time.time()

        # frame FPS estimation
        dt_frame = now - last_frame_ts
        last_frame_ts = now
        if dt_frame > 0:
            fps_hist.append(1.0 / dt_frame)

        # decide whether to run inference this frame
        do_infer = (target_hz <= 0.0) or ((now - last_infer_ts) >= min_period)

        if do_infer:
            t0 = time.time()
            # Inference
            results = model(frame, conf=args.conf, max_det=args.max_det, verbose=False)
            boxes_xyxy, confs, clses = [], [], []
            kpts = []
            if results and results[0].boxes is not None and len(results[0].boxes) > 0:
                b = results[0].boxes
                # Move tensors to CPU safely
                xyxy = b.xyxy.detach().cpu().numpy().astype(int)
                conf = b.conf.detach().cpu().numpy()
                cls  = b.cls.detach().cpu().numpy().astype(int)
                boxes_xyxy = xyxy.tolist()
                confs = conf.tolist()
                clses = cls.tolist()
                # Keypoints (pose 모델일 경우)
                kobj = getattr(results[0], "keypoints", None)
                if kobj is not None:
                    try:
                        karr = kobj.xy
                        karr = karr.detach().cpu().numpy() if hasattr(karr, "detach") else np.array(karr)
                        # shape: [N, K, 2]
                        for inst in karr:
                            kpts.append([(float(p[0]), float(p[1])) for p in inst])
                    except Exception:
                        kpts = [[] for _ in boxes_xyxy]
                else:
                    kpts = [[] for _ in boxes_xyxy]

            overlay = draw_detections(frame, boxes_xyxy, confs, clses, names, kpts=kpts)
            infer_time = time.time() - t0
            last_overlay = overlay
            last_infer_ts = now
            if infer_time > 0:
                ihz_hist.append(1.0 / infer_time)
        else:
            overlay = last_overlay if last_overlay is not None else frame

        # HUD
        hud = overlay.copy()
        avg_fps = np.mean(fps_hist) if fps_hist else 0.0
        avg_ihz = np.mean(ihz_hist) if ihz_hist else 0.0
        hud_lines = [
            f"Target Hz: {target_hz:.2f}  (press '+' / '-' to change)",
            f"Frame FPS: {avg_fps:5.1f}",
            f"Infer Hz:  {avg_ihz:5.1f}",
            f"Conf thr:  {args.conf:.2f}   Max det: {args.max_det:d}",
            f"Device: {device}",
        ]
        if show_help:
            hud_lines.append("Keys: q=quit, +=Hz up, -=Hz down, h=toggle HUD")

        x0, y0 = 10, 22
        for i, line in enumerate(hud_lines):
            y = y0 + i * 20
            # cv2.putText(hud, line, (x0, y),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            # cv2.putText(hud, line, (x0, y),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            # HUD 폰트도 살짝 더 작게
            cv2.putText(hud, line, (x0, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(hud, line, (x0, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)


        cv2.imshow(args.window, hud)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key in (ord('+'), ord('=')):  # '=' key is usually shiftless '+'
            target_hz = min(60.0, target_hz + 1.0) if target_hz > 0 else 1.0
            min_period = 0.0 if target_hz <= 0 else (1.0 / target_hz)
        elif key == ord('-'):
            target_hz = max(0.0, target_hz - 1.0)
            min_period = 0.0 if target_hz <= 0 else (1.0 / target_hz)
        elif key == ord('h'):
            show_help = not show_help

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
