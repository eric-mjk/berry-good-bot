# berry_kinematics_control/kin_utils.py
import pathlib, numpy as np

# # ——— spatialmath.base.graphics 더미 모듈로 주입 ———
# import types, sys
# _gfx = types.ModuleType("spatialmath.base.graphics")

# # 모든 함수가 아무 일도 하지 않는 람다로 대체
# _noop = lambda *_, **__: None
# for _name in (
#         "plotvol3", "axes_logic",
#         "plot_ellipse",        # ← 추가
#         "plot_ellipsoid",      # (선택) 나중에 또 필요할 수 있음
#         "plot_sphere",         # (선택)
#         "plot_arrow"           # (선택)
# ):
#     setattr(_gfx, _name, _noop)

# sys.modules["spatialmath.base.graphics"] = _gfx

# pip install "roboticstoolbox-python==1.1.0" "spatialmath-python==1.1.8" "matplotlib<3.8"

# pip install "scipy<1.12" --upgrade    # 1.11.4가 내려올 것
# pip install xacro

# 이 명령어로 설치하면 됌.

from roboticstoolbox import Robot
from ikpy.chain import Chain
from ruamel.yaml import YAML

import xacro
import tempfile
import pathlib

def build_models(urdf_path: str):
    """URDF → (RTB_Robot, IKPy_Chain, joint_names)"""
    if urdf_path.endswith('.xacro'):
        # 1) xacro 처리 → 순수 URDF XML 문자열 얻기
        doc      = xacro.process_file(urdf_path)
        urdf_xml = doc.toxml()          # 일단 문자열로 만든 뒤

        # --- 안전하게 DOM 수정 ---
        import xml.etree.ElementTree as ET, math
        root = ET.fromstring(urdf_xml)

        # continuous → revolute, limit 보정
        for joint in root.findall('.//joint[@type="continuous"]'):
            joint.set('type', 'revolute')

            limit = joint.find('limit')
            if limit is None:
                limit = ET.SubElement(joint, 'limit')

            # 이미 있으면 건드리지 않음
            if 'lower' not in limit.attrib:
                limit.set('lower', str(-math.pi))
            if 'upper' not in limit.attrib:
                limit.set('upper', str( math.pi))

        # 다시 문자열로 직렬화
        urdf_xml = ET.tostring(root, encoding='unicode')

        # 4) 최종 URDF를 임시 .urdf 파일에 기록
        tf = tempfile.NamedTemporaryFile(mode='w+', delete=False, suffix='.urdf')
        tf.write(urdf_xml)
        tf.flush()
        path_for_load = tf.name
    else:
        # 이미 .urdf 파일이라면 그대로
        path_for_load = urdf_path

    # 5) RTB & IKPy 모두 파일 경로로 로드
    rtb_robot = Robot.URDF(path_for_load)
    ik_chain  = Chain.from_urdf_file(path_for_load)

    joints    = [j.name for j in rtb_robot.q]

    return rtb_robot, ik_chain, joints

def load_named_poses(yaml_path: str):
    yaml = YAML(typ="safe")
    return yaml.load(pathlib.Path(yaml_path))

def damped_pinv(J: np.ndarray, lam: float = 0.05):
    """Damped-least-squares pseudoinverse"""
    JTJ = J.T @ J
    return np.linalg.inv(JTJ + (lam ** 2) * np.eye(J.shape[1])) @ J.T

def lspb_interpolate(q0, qf, n_steps=100):
    """단순 선형 + cosine-blend 궤적 (0-1 구간 S-curve)"""
    q0, qf = np.asarray(q0), np.asarray(qf)
    for k in range(n_steps + 1):
        s = k / n_steps
        s_blend = 0.5 - 0.5 * np.cos(np.pi * s)
        yield (1 - s_blend) * q0 + s_blend * qf
