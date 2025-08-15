# hsv_picker.py
import sys, cv2, numpy as np

IMAGE = sys.argv[1] if len(sys.argv) > 1 else "image.jpg"

img = cv2.imread(IMAGE)
if img is None:
    raise SystemExit(f"이미지를 못 읽었습니다: {IMAGE}")

disp = img.copy()
freeze = False
last_pt = None

def bgr_to_hsv_info(bgr):
    # OpenCV HSV: H:0-179, S:0-255, V:0-255
    hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0,0]
    h,s,v = int(hsv[0]), int(hsv[1]), int(hsv[2])
    # 사람이 보기 쉬운 단위로도 보이게
    h_deg = h * 2                   # 0~360°
    s_pct = round(s / 255 * 100, 1) # 0~100%
    v_pct = round(v / 255 * 100, 1) # 0~100%
    return (h,s,v), (h_deg, s_pct, v_pct)

def on_mouse(event, x, y, flags, param):
    global disp, freeze, last_pt
    if not freeze and (event == cv2.EVENT_MOUSEMOVE or event == cv2.EVENT_LBUTTONDOWN):
        last_pt = (x, y)
        redraw()

    if event == cv2.EVENT_LBUTTONDOWN:
        freeze = not freeze  # 클릭으로 일시정지/해제
        redraw()

def redraw():
    global disp, last_pt
    disp = img.copy()
    if last_pt is None:
        return
    x,y = last_pt
    h, w = img.shape[:2]
    if not (0 <= x < w and 0 <= y < h):
        return

    bgr = img[y, x].tolist()
    (hcv, scv, vcv), (h_deg, s_pct, v_pct) = bgr_to_hsv_info(bgr)

    # 화면 오버레이
    cv2.drawMarker(disp, (x,y), (255,255,255), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=1)
    text1 = f"HSV (OpenCV): H={hcv} S={scv} V={vcv}"
    text2 = f"HSV (deg/%): H={h_deg}°  S={s_pct}%  V={v_pct}%"
    box_w = 520
    cv2.rectangle(disp, (10,10), (10+box_w, 60), (0,0,0), -1)
    cv2.putText(disp, text1, (20,35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(disp, text2, (20,55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
    print(f"(x={x}, y={y})  BGR={bgr}  OpenCV_HSV={hcv, scv, vcv}  deg/%={h_deg}°, {s_pct}%, {v_pct}%")

cv2.namedWindow("HSV Picker", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("HSV Picker", on_mouse)
redraw()

print("마우스 이동: HSV 표시, 왼쪽 클릭: 일시정지/해제, q: 종료")
while True:
    cv2.imshow("HSV Picker", disp)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        break

cv2.destroyAllWindows()
