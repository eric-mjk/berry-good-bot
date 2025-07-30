#!/usr/bin/env python3
"""
pip install pynput 필요

keyboard_servo.py  ──  BerryBot EEF visual-servo 디버그용 (ROS 불필요)

• WASD : +Y/–Y, +X/–X   (Turtle Bot 스타일)
• Q/E  : –Z / +Z
• U/O  : +Roll / –Roll
• I/K  : +Pitch / –Pitch
• J/L  : +Yaw / –Yaw
Esc    : 종료
"""
import numpy as np
from pynput import keyboard          # 실시간 키 입력 :contentReference[oaicite:1]{index=1}
from ikpy.chain import Chain
from roboticstoolbox import Robot    # jacob0 구현 :contentReference[oaicite:2]{index=2}
from transforms3d.euler import mat2euler, euler2mat
import time, pathlib, sys

# ─────────────────────────────────────────────────────────────────────────────
URDF_PATH = pathlib.Path(__file__).parent / "../urdf/berrybot.urdf"
ACTIVE_MASK = [False, True, True, True, True, True]      # base link 고정 :contentReference[oaicite:3]{index=3}
DT   = 0.02        # [s] 시뮬레이션 주기 (50 Hz)
VLIN = 0.01        # 선속도 스텝  [m/s ≈ m/step]
VANG = 0.05        # 각속도 스텝  [rad/s ≈ rad/step]
DAMP = 0.05        # DLS λ :contentReference[oaicite:4]{index=4}

# ─── 로봇 모델 로드 ──────────────────────────────────────────────────────────
rtb_robot = Robot.URDF(str(URDF_PATH))
chain     = Chain.from_urdf_file(str(URDF_PATH), active_links_mask=ACTIVE_MASK)

q_cur   = np.array([-0.10, 0.61, -2.443, 0.172, 0.0])    # Home pose
twist_cmd = np.zeros(6)                                  # [vx vy vz ωx ωy ωz]

# ─── Damped-Pseudo-Inverse ---------------------------------------------------
def damped_pinv(J, lam=DAMP):
    return np.linalg.inv(J.T @ J + (lam**2)*np.eye(J.shape[1])) @ J.T

# ─── 키보드 핸들러 -----------------------------------------------------------
KEY_MAP = {
    'w': ( 1, +VLIN), 's': ( 1, -VLIN),                  # +Y / –Y
    'a': ( 0, -VLIN), 'd': ( 0, +VLIN),                  # –X / +X
    'e': ( 2, +VLIN), 'q': ( 2, -VLIN),                  # +Z / –Z
    'u': ( 3, +VANG), 'o': ( 3, -VANG),                  # +R / –R
    'i': ( 4, +VANG), 'k': ( 4, -VANG),                  # +P / –P
    'j': ( 5, +VANG), 'l': ( 5, -VANG),                  # +Y / –Y (yaw)
}

def on_press(key):
    try:
        k = key.char.lower()
        if k in KEY_MAP:
            axis, step = KEY_MAP[k]
            twist_cmd[axis] = step
    except AttributeError:
        if key == keyboard.Key.esc:
            print("\n[ESC] 종료")
            return False   # Listener stop

def on_release(key):
    try:
        k = key.char.lower()
        if k in KEY_MAP:
            axis, _ = KEY_MAP[k]
            twist_cmd[axis] = 0.0
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()
print("▶ 키보드를 눌러 BerryBot EEF 를 움직여보세요 (ESC 종료)\n")

# ─── 메인 루프 ---------------------------------------------------------------
while listener.running:
    J = rtb_robot.jacob0(q_cur)                           # 6×5 Jacobian
    qdot = damped_pinv(J) @ twist_cmd                     # :contentReference[oaicite:5]{index=5}
    q_cur += qdot * DT

    # FK → 현재 EEF pose
    fk = chain.forward_kinematics(np.r_[0.0, q_cur])
    pos = fk[:3, 3]
    rpy = np.degrees(mat2euler(fk[:3, :3], axes='sxyz'))

    sys.stdout.write("\rpos [m] = %+.3f %+.3f %+.3f | rpy [deg] = %+6.1f %+6.1f %+6.1f   "
                     % (*pos, *rpy))
    sys.stdout.flush()
    time.sleep(DT)
