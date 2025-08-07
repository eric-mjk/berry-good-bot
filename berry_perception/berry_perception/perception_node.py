#!/usr/bin/env python3
import rclpy, math, cv2, torch, numpy as np, os
from rclpy.node            import Node
from rclpy.action          import ActionServer
from sensor_msgs.msg       import Image, CameraInfo
from geometry_msgs.msg     import TransformStamped, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from tf2_ros               import StaticTransformBroadcaster, Buffer, TransformListener
import tf_transformations   as tft
from cv_bridge             import CvBridge
from ultralytics           import YOLO
from berry_interface.action import DetectStrawberry

from ament_index_python.packages import get_package_share_directory

class PerceptionNode(Node):
    def __init__(self):
        # super().__init__('perception_node')
        super().__init__(
            "perception_node",
            # YAML-override 에서 온 모든 키를 자동 선언
            automatically_declare_parameters_from_overrides=True
        )

        # ────── Parameters ───────────────────────────────────────────
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('eef_frame',     'link5')
        # self.declare_parameter('camera_to_eef', [0.0, 0.08, 0.045,  math.pi/180*30, -math.pi/2, -math.pi/2]) # xyz + rpy
        self.declare_parameter('camera_to_eef', [0.0, 0.09, 0.045,  math.pi/180*0, -math.pi/2, -math.pi/2]) # xyz + rpy
        cam2eef = self.get_parameter('camera_to_eef').value
        self.publish_static_tf(cam2eef)

        # ────── Subscribers / utils ─────────────────────────────────
        self.bridge   = CvBridge()
        self.rgb_sub  = self.create_subscription(Image,  '/camera/color/image_raw', self.rgb_cb, 10)
        self.dep_sub  = self.create_subscription(Image,  '/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_cb, 10)

        self.rgb_img   = None
        self.depth_img = None
        self.cam_info  = None

        # TF buffer for base_link 변환
        self.tfb  = Buffer()
        self.tfl  = TransformListener(self.tfb, self)

        # RViz marker publisher
        self.marker_pub = self.create_publisher(Marker, 'strawberry_mark',  10)
        self.appr_pub   = self.create_publisher(Marker, 'approach_mark',    10)

        # ────── YOLOv8 초기화 ───────────────────────────────────────
        pkg_share  = get_package_share_directory('berry_perception')
        model_path = os.path.join(pkg_share, 'model', 'model1.pt')
        self.get_logger().info(f"Loading YOLO model from {model_path}")
        self.model = YOLO(model_path)
        self.model.model.to('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"YOLO device: {self.model.device}")

        # ────── Action Server ──────────────────────────────────────
        self._action_server = ActionServer(
            self, DetectStrawberry, 'detect_strawberry', self.execute_cb)

    # ----------------- Static TF ----------------------------------
    def publish_static_tf(self, xyzrpy):
        br = StaticTransformBroadcaster(self)
        t  = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('eef_frame').value
        t.child_frame_id  = self.get_parameter('camera_frame').value
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyzrpy[:3]
        # q = tft.quaternion_from_euler(*xyzrpy[3:])
        # t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # ── (1) 원래 RPY → 행렬 ────────────────────────────────
        base_rot = tft.euler_matrix(*xyzrpy[3:])          # 4×4

        # ── (2) 보정: 초록(Y)축 기준 +30° 앞쪽으로 기울이기 ──
        tilt_rad = math.radians(35.0)                     # 필요 시 −값으로 바꿔보세요
        corr_mat = tft.rotation_matrix(tilt_rad, (0, 1, 0))

        # ── (3) 두 행렬 곱 → 최종 쿼터니언 ────────────────────
        final_q  = tft.quaternion_from_matrix(
                      tft.concatenate_matrices(base_rot, corr_mat))
        t.transform.rotation = Quaternion(
           x=final_q[0], y=final_q[1], z=final_q[2], w=final_q[3])
        br.sendTransform(t)

    # ----------------- Subs ---------------------------------------
    def rgb_cb(self, msg):   self.rgb_img   = msg
    def depth_cb(self, msg): self.depth_img = msg
    def info_cb(self, msg):  self.cam_info  = msg

    # ----------------- Main Action --------------------------------
    async def execute_cb(self, goal_handle):
        offset_z = goal_handle.request.offset_z
        # 최신 frame 올 때까지 대기
        while self.rgb_img is None or self.depth_img is None or self.cam_info is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        cv_img = self.bridge.imgmsg_to_cv2(self.rgb_img, 'bgr8')
        results = self.model.predict(source=cv_img, conf=0.4, save=False, verbose=False)
        if not results or not results[0].boxes:
            self.get_logger().warn('No strawberry detected')
            goal_handle.abort()
            return DetectStrawberry.Result()

        # 가장 높은 confidence 선택
        boxes = results[0].boxes
        best  = boxes[boxes.conf.argmax()]
        x1, y1, x2, y2 = map(int, best.xyxy[0])
        cx, cy = int((x1+x2)/2), int((y1+y2)/2)

        # z값(dept) 확보
        depth = self.bridge.imgmsg_to_cv2(self.depth_img, 'passthrough')[cy, cx] / 1000.0  # mm→m
        if depth == 0.0:
            goal_handle.abort()
            return DetectStrawberry.Result()

        # 픽셀→3D (pin-hole)
        fx = self.cam_info.k[0];  fy = self.cam_info.k[4]
        cx0 = self.cam_info.k[2]; cy0 = self.cam_info.k[5]
        X = (cx - cx0) * depth / fx
        Y = (cy - cy0) * depth / fy
        Z = depth

        st_pose_cam = PoseStamped()
        st_pose_cam.header.stamp = self.rgb_img.header.stamp
        st_pose_cam.header.frame_id = self.get_parameter('camera_frame').value
        st_pose_cam.pose.position = Point(x=X, y=Y, z=Z)
        st_pose_cam.pose.orientation.w = 1.0  # no orientation

        # feedback
        goal_handle.publish_feedback(
            DetectStrawberry.Feedback(strawberry_pose_cam=st_pose_cam))

        # 접근점 = Z축 앞으로 offset
        appr_cam = PoseStamped()
        appr_cam.header = st_pose_cam.header
        appr_cam.pose.position = Point(x=X, y=Y, z=Z - offset_z)
        appr_cam.pose.orientation.w = 1.0

        # 카메라→base_link TF 변환
        try:
            tf_cam2base = self.tfb.lookup_transform(
                'base_link', st_pose_cam.header.frame_id,
                rclpy.time.Time())  # 가장 최근
            mat = tft.concatenate_matrices(
                tft.translation_matrix((tf_cam2base.transform.translation.x,
                                        tf_cam2base.transform.translation.y,
                                        tf_cam2base.transform.translation.z)),
                tft.quaternion_matrix(
                    (tf_cam2base.transform.rotation.x,
                     tf_cam2base.transform.rotation.y,
                     tf_cam2base.transform.rotation.z,
                     tf_cam2base.transform.rotation.w)))
            p = np.array([appr_cam.pose.position.x,
                          appr_cam.pose.position.y,
                          appr_cam.pose.position.z, 1.0])
            p_base = mat @ p
        except Exception as e:
            self.get_logger().error(f"TF error: {e}")
            goal_handle.abort()
            return DetectStrawberry.Result()

        appro_pose_base = PoseStamped()
        appro_pose_base.header.frame_id = 'base_link'
        appro_pose_base.header.stamp = self.get_clock().now().to_msg()
        appro_pose_base.pose.position = Point(*p_base[:3])
        appro_pose_base.pose.orientation.w = 1.0

        # RViz 마커 두 개 ---------------------------------------------------
        self.publish_marker(st_pose_cam, id=0, color=(0.0, 1.0, 0.0))  # green
        self.publish_marker(appro_pose_base, id=1, color=(1.0, 0.0, 0.0))  # red

        goal_handle.succeed()
        return DetectStrawberry.Result(approach_pose_base=appro_pose_base)

    # ----------------- RViz marker util ----------------------------
    def publish_marker(self, pose, id, color):
        m = Marker()
        m.header = pose.header
        m.ns = 'berry'
        m.id = id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = m.scale.y = m.scale.z = 0.03
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 0.8
        pub = self.marker_pub if id == 0 else self.appr_pub
        pub.publish(m)

def main():
    rclpy.init()
    rclpy.spin(PerceptionNode())
    rclpy.shutdown()
