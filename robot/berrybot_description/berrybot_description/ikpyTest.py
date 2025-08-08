# ikpyTest.py ---------------------------------------------------------------
import numpy as np
from ikpy.chain import Chain
# IKPy 쪽 geometry 모듈 대신 transforms3d 사용
from transforms3d.euler import mat2euler       # ← 새로 import

# ── URDF 로드 ──────────────────────────────────────────────────────────────
URDF_PATH = "../urdf/berrybot.urdf"          # 같은 폴더에 둘 것(또는 절대경로 지정)
chain = Chain.from_urdf_file(
    URDF_PATH,
    # 첫 link(base_link)와 마지막 dummy-link(EEF)는 비활성화
    active_links_mask=[False,   # base_link (fixed)
                       True,    # joint1
                       True,    # joint2
                       True,    # joint3
                       True,    # joint4
                       True]    # joint5
)
# chain = Chain.from_urdf_file(URDF_PATH)
print("len(chain.links) : ", len(chain.links))
print("[l.name for l in chain.links] : ", [l.name for l in chain.links])

# ── Named poses (joint1 ~ joint5) ──────────────────────────────────────────
poses = {
    "home":   [-0.30, -1.95, 2.3, 1.5, 0.0],
    "ready":  [-0.53, -1.77, 1.40, -0.68, 0.0],
    "basket": [-0.15, -1.11, -1.20, 0.73, 0.0],
    "test": [-0.20,  -0.0, -0.0, 0.0, 0.0],
    # [-0.53, -1.77, 1.40, -0.68, 0.0]
}

print("=== BerryBot FK ↔ IK 테스트 (ikpy) ===\n")

for name, q_active in poses.items():
    # ikpy.forward_kinematics/ik 는 모든 링크 길이만큼의 배열 필요
    # [base(0)] + 5개 active + [EEF(0)]
    q_full = [0.0] + q_active

    # ① Forward Kinematics --------------------------------------------------
    fk_mat = chain.forward_kinematics(q_full)
    pos = fk_mat[:3, 3]                       # (x, y, z)
    # rot = fk_mat[:3, :3]                      # 3×3 회전행렬
    # # 회전을 roll-pitch-yaw 로 변환(도)
    # rpy = np.degrees(geometry.rotation_matrix_to_euler_angles(fk_mat))
    rot = fk_mat[:3, :3]                      # 3×3 회전행렬
    # 회전을 roll-pitch-yaw 로 변환(도) — 'sxyz' = extrinsic XYZ
    rpy = np.degrees(mat2euler(rot, axes='sxyz'))

    # ② Inverse Kinematics --------------------------------------------------
    ik_full = chain.inverse_kinematics_frame(fk_mat, initial_position=q_full)
    ik_active = np.round(ik_full[1:], 5)    # base/EEF 제외 & 소수 5자리

    # ③ 결과 출력 -----------------------------------------------------------
    print(f"[{name.upper()}]")
    print(f" FK  ▶ pos = {pos.round(4)},  rpy = {rpy.round(2)}°")
    print(f" IK  ▶ joints = {ik_active}\n")
