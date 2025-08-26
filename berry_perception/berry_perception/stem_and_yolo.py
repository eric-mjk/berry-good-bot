# berry_perception/perception/stem_and_yolo.py
import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass

# =========================================================
# ============ BOTTOM-CAM GRIPPER MASKING CONFIG ==========
# YAML 없이 여기서 직접 값을 바꿔서 사용하세요.
#  - 폴리곤은 꼭짓점 4개(x,y) → 총 8개 숫자
#  - 흰색: HSV = (0, 0, 255) / 회색: (0, 0, 200) 등
BOTTOM_MASK_ENABLE: bool = True
BOTTOM_MASK_POLY1: List[int] = [47,0, 126,251, 185,251, 187,209, 221,220, 204,103, 153,0]  # 예) [120,110,  180,90,  210,230,  140,260]
BOTTOM_MASK_POLY2: List[int] = [526,0, 433,254, 380,255, 364,222, 360,112, 409,0]  # 예) [420,105,  480,130,  450,270,  390,240]
BOTTOM_MASK_HSV: Tuple[int,int,int] = (66, 21, 204)
BOTTOM_MASK_FEATHER: int = 5  # 경계 부드럽게(픽셀). 0이면 딱 채움

def get_bottom_mask_polygons() -> List[np.ndarray]:
    """설정값을 np.ndarray 폴리곤 리스트로 변환"""
    polys: List[np.ndarray] = []
    for arr in (BOTTOM_MASK_POLY1, BOTTOM_MASK_POLY2):
        if isinstance(arr, (list, tuple)) and len(arr) >= 8:
            try:
                pts = np.array(arr, dtype=np.int32).reshape(-1, 2)
                polys.append(pts)
            except Exception:
                pass
    return polys

# -------------------- PREPROCESS: paint polygons with HSV --------------------
def hsv_to_bgr(hsv_triplet: Tuple[int,int,int]) -> Tuple[int,int,int]:
    """HSV(0~179,0~255,0~255) → BGR 튜플"""
    h, s, v = [int(x) for x in hsv_triplet]
    pix = np.uint8([[[h % 180, max(0, min(255, s)), max(0, min(255, v))]]])
    bgr = cv2.cvtColor(pix, cv2.COLOR_HSV2BGR)[0, 0]
    return (int(bgr[0]), int(bgr[1]), int(bgr[2]))

def paint_polygons_with_hsv(
    bgr: np.ndarray,
    polygons: List[np.ndarray],
    hsv_color: Tuple[int,int,int] = (0, 0, 255),
    feather: int = 0,
    alpha: float = 1.0
) -> Tuple[np.ndarray, np.ndarray]:
    """
    polygons: [Nx2 int32, ...]
    hsv_color: (H,S,V). 기본 흰색.
    feather: 경계 부드럽게(픽셀). 0이면 바로 채움.
    alpha: 1.0 대체, (0,1) 블렌드
    return: (painted_bgr, mask_uint8)
    """
    H, W = bgr.shape[:2]
    mask = np.zeros((H, W), np.uint8)
    for poly in polygons:
        if poly is None: 
            continue
        pts = np.array(poly, dtype=np.int32).reshape(-1, 2)
        if pts.shape[0] >= 3:
            cv2.fillPoly(mask, [pts], 255)

    bgr_color = np.array(hsv_to_bgr(hsv_color), dtype=np.float32)
    out = bgr.copy().astype(np.float32)

    if feather and feather > 0:
        k = int(feather) * 2 + 1  # 가우시안은 홀수 커널
        soft = cv2.GaussianBlur(mask, (k, k), 0).astype(np.float32) / 255.0
        for c in range(3):
            out[:, :, c] = soft * (alpha * bgr_color[c] + (1 - alpha) * out[:, :, c]) + (1 - soft) * out[:, :, c]
        out = np.clip(out, 0, 255).astype(np.uint8)
    else:
        sel = mask > 0
        out[sel] = (alpha * bgr_color + (1 - alpha) * out[sel]).astype(np.uint8)
    return out, mask

# -------------------- YOLO helper --------------------
def choose_track(prev_bbox: Optional[List[int]],
                 xyxy_list: List[List[int]],
                 conf_list: List[float]) -> Optional[int]:
    """prev bbox가 있으면 IoU 최대(>0.1) 우선, 아니면 conf 최대. 낮으면 센터거리 최소."""
    if not xyxy_list:
        return None
    if prev_bbox is None:
        return int(np.argmax(conf_list))
    ax1, ay1, ax2, ay2 = prev_bbox
    a_area = max(0, ax2-ax1) * max(0, ay2-ay1)
    best_iou, best_idx = -1.0, -1
    for i, (bx1,by1,bx2,by2) in enumerate(xyxy_list):
        iw = max(0, min(ax2, bx2) - max(ax1, bx1))
        ih = max(0, min(ay2, by2) - max(ay1, by1))
        inter = iw * ih
        b_area = max(0, bx2-bx1) * max(0, by2-by1)
        union = a_area + b_area - inter + 1e-9
        iou = inter / union
        if iou > best_iou:
            best_iou, best_idx = iou, i
    if best_iou > 0.1:
        return best_idx
    # fallback: center distance
    ax = 0.5*(ax1+ax2); ay=0.5*(ay1+ay2)
    d = [((0.5*(b[0]+b[2]) - ax)**2 + (0.5*(b[1]+b[3]) - ay)**2)**0.5 for b in xyxy_list]
    return int(np.argmin(d))

# -------------------- STEM detector --------------------
def _clamp_roi(x1, y1, x2, y2, w, h):
    x1 = max(0, min(w-1, int(x1)))
    x2 = max(0, min(w-1, int(x2)))
    y1 = max(0, min(h-1, int(y1)))
    y2 = max(0, min(h-1, int(y2)))
    if x2 <= x1: x2 = min(w-1, x1+1)
    if y2 <= y1: y2 = min(h-1, y1+1)
    return x1, y1, x2, y2


def _rect_intersects(a: Tuple[int,int,int,int], b: Tuple[int,int,int,int]) -> bool:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    iw = max(0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0, min(ay2, by2) - max(ay1, by1))
    return (iw > 0) and (ih > 0)

def _rect_iou(a: Tuple[int,int,int,int], b: Tuple[int,int,int,int]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    iw = max(0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    if inter <= 0:
        return 0.0
    a_area = max(0, ax2-ax1) * max(0, ay2-ay1)
    b_area = max(0, bx2-bx1) * max(0, by2-by1)
    return float(inter) / float(a_area + b_area - inter + 1e-9)
 
# =========================================================
# -------------------- STRAWBERRY SEGMENTATION -----------
def _as_xyxy(rect: Tuple[int,int,int,int]) -> Tuple[int,int,int,int]:
    """rect가 (x,y,w,h) 또는 (x1,y1,x2,y2)이어도 안전하게 (x1,y1,x2,y2)로 변환"""
    x1,y1,x2,y2 = rect
    if x2 <= x1 or y2 <= y1:  # (x,y,w,h)로 추정
        x,y,w,h = rect
        return int(x), int(y), int(x+w), int(y+h)
    return int(x1), int(y1), int(x2), int(y2)

def _largest_contour(bin_mask: np.ndarray):
    """이진 마스크에서 가장 큰 컨투어 반환 (None 가능)"""
    cnts, _ = cv2.findContours(bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    cnt = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(cnt) <= 0:
        return None
    return cnt

def segment_strawberry(
    bgr: np.ndarray,
    roi_rect: Tuple[int,int,int,int],
    hsv_ranges_ry: List[Tuple[Tuple[int,int,int], Tuple[int,int,int]]] = (
        # 빨강(저Hue) + 빨강(고Hue) + 노랑 계열
        ((0, 100, 130), (15, 255, 220)),   # red low
        ((0, 200, 100), (15, 255, 200)),   # red high
        ((15,  200, 110), (25,  255, 155)),   # yellowish
        ((0, 150, 90), (12, 210, 140)),   # red apple
    ),
    morph_open: int = 3,
    morph_close: int = 5,
) -> Dict:
    """
    ROI 내에서 HSV(red~yellow) 기반으로 딸기 영역을 바이너리화한 뒤
    가장 큰 연결 성분의 마스크와 컨투어를 반환.
    반환:
      {
        'ok': bool,
        'mask': HxW uint8 (0/255),
        'contour': Nx2 int (글로벌 좌표) 또는 None,
        'roi': (x1,y1,x2,y2)
      }
    """
    H, W = bgr.shape[:2]
    x1,y1,x2,y2 = _as_xyxy(roi_rect)
    x1,y1,x2,y2 = _clamp_roi(x1,y1,x2,y2, W, H)
    if (x2-x1) <= 1 or (y2-y1) <= 1:
        return {'ok': False, 'mask': np.zeros((H,W), np.uint8), 'contour': None, 'roi': (x1,y1,x2,y2)}

    roi = bgr[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo,hi in hsv_ranges_ry:
        mask |= cv2.inRange(hsv, np.array(lo,np.uint8), np.array(hi,np.uint8))

    if morph_open > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((morph_open,morph_open), np.uint8), iterations=1)
    if morph_close > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((morph_close,morph_close), np.uint8), iterations=1)

    # 가장 큰 컨투어 선택
    cnt = _largest_contour(mask)
    full_mask = np.zeros((H,W), np.uint8)
    if cnt is None:
        return {'ok': False, 'mask': full_mask, 'contour': None, 'roi': (x1,y1,x2,y2)}

    # ROI 좌표 → 글로벌로 이동
    cnt_glob = cnt.reshape(-1,2).astype(np.int32)
    cnt_glob[:,0] += x1
    cnt_glob[:,1] += y1

    # 글로벌 마스크 생성
    cv2.drawContours(full_mask, [cnt_glob.reshape(-1,1,2)], -1, 255, thickness=cv2.FILLED)

    return {
        'ok': True,
        'mask': full_mask,
        'contour': cnt_glob,
        'roi': (x1,y1,x2,y2)
    }


# =========================================================
# -------------------- YOLO SELECTION (by SEG ROI similarity)
def _bbox_from_contour_glob(contour_glob: np.ndarray) -> Tuple[int,int,int,int]:
    """
    contour_glob: Nx2 (global coords)
    returns (x1,y1,x2,y2)
    """
    if contour_glob is None or contour_glob.size == 0:
        return None
    xs = contour_glob[:,0]
    ys = contour_glob[:,1]
    x1 = int(np.min(xs)); y1 = int(np.min(ys))
    x2 = int(np.max(xs)); y2 = int(np.max(ys))
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2, y2)

def select_yolo_by_roi_similarity(
    bgr: np.ndarray,
    gr_xyxy: Tuple[int,int,int,int],
    xyxy_list: List[List[int]],
    conf_list: List[float],
    score_thresh: float = 0.30
) -> Tuple[Optional[int], float, Optional[Tuple[int,int,int,int]], Dict, float]:

    """
    1) gripper ROI를 사용해 segment_strawberry 수행
    2) 세그멘트 마스크로부터 ROI(사각형) 생성
    3) gripper ROI와 '겹치는' YOLO 박스들 중에서,
       점수 score = IoU(seg_roi, yolo_box) + conf 가 가장 큰 박스 선택
       (동률 시 IoU 큰 박스, 그 다음 conf 큰 박스 우선)
    4) 최종 best_score가 score_thresh 이하이면 유효한 YOLO 없음으로 판단
 

    반환:
      (selected_idx, best_iou, seg_roi_xyxy, seg_dict, best_score)
    """
    # 세그먼트 먼저 수행
    seg = segment_strawberry(bgr, gr_xyxy)
    seg_roi = None
    if seg.get('contour', None) is not None:
        seg_roi = _bbox_from_contour_glob(seg['contour'])
    # 세그 실패 시 gripper ROI 자체를 비교 기준으로 사용
    if seg_roi is None:
        seg_roi = tuple(gr_xyxy)

    # 후보: gripper ROI와 겹치는 박스만
    cand = []
    for i, bb in enumerate(xyxy_list):
        bb_t = tuple(map(int, bb))
        if _rect_intersects(bb_t, gr_xyxy):
            conf = float(conf_list[i]) if (conf_list is not None and i < len(conf_list)) else 0.0
            cand.append((i, bb_t, conf))

    # if not cand:
    #     return None, 0.0, seg_roi, seg
    if not cand:
        return None, 0.0, seg_roi, seg, 0.0

    # # IoU 최대 + tie-break by conf
    # best = (-1, -1.0, -1.0)  # (idx, iou, conf)
    # score = IoU + conf 최대 + tie-break (IoU -> conf)
    best = (-1, -1.0, -1.0, -1.0)  # (idx, score, iou, conf)
    for (i, bb_t, conf) in cand:
        iou = _rect_iou(bb_t, seg_roi)
        # # 우선 IoU 큰 것, 같으면 conf 큰 것
        score = iou + conf
        # 우선 score 큰 것, 같으면 iou 큰 것, 또 같으면 conf 큰 것
        if (score > best[1]) or \
           (np.isclose(score, best[1]) and iou > best[2]) or \
           (np.isclose(score, best[1]) and np.isclose(iou, best[2]) and conf > best[3]):
            best = (i, score, iou, conf)

    selected_idx = best[0] if best[0] >= 0 else None
    best_score = best[1] if best[1] >= 0 else 0.0
    best_iou = best[2] if best[2] >= 0 else 0.0

    # 임계값 이하인 경우 무효 처리
    if selected_idx is None or best_score <= float(score_thresh):
        return None, float(best_iou), seg_roi, seg, float(best_score)

    return selected_idx, float(best_iou), seg_roi, seg, float(best_score)

