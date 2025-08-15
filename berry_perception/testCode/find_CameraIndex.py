import cv2

# 테스트할 인덱스 범위 (0~10까지 시도)
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"카메라 발견: 인덱스 {i}")
        cap.release()
