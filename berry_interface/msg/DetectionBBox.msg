# msg/DetectionBBox.msg
std_msgs/Header header
string   camera_name     # "rgb" or "rgbd"
float32  conf            # best detection confidence
int32    x1
int32    y1
int32    x2
int32    y2
float32  cx              # bbox center (pixels)
float32  cy
float32  depth_m         # meters; NaN if not available (e.g., RGB)

# --- Pose / Keypoints (YOLO pose 출력) ---
# Pixel 좌표 (이미지 좌표계, 단위: px). 길이는 kpt_count 와 동일.
uint32   kpt_count
float32[] kpt_x          # size = kpt_count
float32[] kpt_y          # size = kpt_count
float32[] kpt_conf       # size = kpt_count (없으면 -1로 채움)

# RGBD일 때만 유효한 키포인트 3D (카메라 프레임, 단위: m).
# 프레임은 RGBD의 camera_frame(파라미터) 또는 RGB의 rgb_camera_frame를 따른다.
# 깊이를 못 구한 키포인트는 NaN으로 채움.
geometry_msgs/Point[] kpt_cam_xyz
