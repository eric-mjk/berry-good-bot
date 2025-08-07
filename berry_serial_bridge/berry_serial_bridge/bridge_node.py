#!/usr/bin/env python3
"""
berry_serial_bridge : RViz <-> UNO & MEGA 시리얼 브리지 노드
1) 두 보드가 모두 "Homing 완료" 메세지를 보낼 때까지 대기
2) 사용자가 <Enter> 키를 치면 /joint_states 구독 시작
3) 변환 후 보드로 명령, 보드 피드백을 /current_joint_states(10 Hz) 발행
"""
import math, sys, threading, time
from typing import Dict

import rclpy
from rclpy.node import Node
import rclpy.logging
from sensor_msgs.msg import JointState
import serial                                          # pyserial
import pygame                     # 🔸 추가
import os


# ─────────────── 시리얼 래퍼 ────────────────
class BoardSerial:
    def __init__(self, name, port, baud, cb):
        self.name, self.port, self.baud, self.cb = name, port, baud, cb
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.logger = rclpy.logging.get_logger(f"Serial.{name}")
        self._stop = threading.Event()
        self._rx_th = threading.Thread(target=self._rx_loop,
                                       daemon=True).start()
    def _rx_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            if self.ser.in_waiting:
                b = self.ser.read()
                if b == b'\n':
                    line = buf.decode(errors='ignore').strip()
                    # print(f"[{self.name}] ◀ {line}")
                    self.logger.info(f"◀(RX) {line}")
                    self.cb(line)
                    buf.clear()
                else:
                    buf.extend(b)
            else:
                time.sleep(0.01)
    def send(self, s: str):
        self.ser.write(s.encode())
        self.logger.info(f"▶(TX) {s.strip()}")
    def close(self):
        self._stop.set()
        self.ser.close()
        self.logger.info("Serial closed")

# ─────────────── pygame 모니터 스레드 ────────────────
class PygameDisplay(threading.Thread):
    """작은 텍스트 대시보드.  Return 키 → enter_event set()"""
    def __init__(self):
        super().__init__(daemon=True)
        # pygame.init()
        # self.screen = pygame.display.set_mode((640, 480))
        # pygame.display.set_caption("Berry Serial Bridge Monitor")
        # self.font = pygame.font.Font(None, 24)

        # ⚠️ GL 컨텍스트는 run() 안, 동일 스레드에서 생성한다
        self.screen = None
        self.font   = None

        # public 값들
        self.status     = "Waiting for homing..."
        self.rx_uno     = {}
        self.rx_mega    = {}
        self.tx_uno     = {}
        self.tx_mega    = {}
        self.topic_vals = {}
        self.curr_js    = {}          # ← 피드백용
        # thread control
        self._stop  = threading.Event()
        self.enter_event = threading.Event()
        self.start()

    def run(self):
        # ── 여기서 창‧컨텍스트 생성 ──
        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("Berry Serial Bridge Monitor")
        self.font = pygame.font.Font(None, 24)

        clock = pygame.time.Clock()
        while not self._stop.is_set():
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self._stop.set()
                elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_RETURN:
                    self.enter_event.set()
            self._draw()
            clock.tick(20)  # 20 FPS
        pygame.quit()

    def _draw(self):
        # 창 생성 전이면 아무것도 그리지 않음
        if self.screen is None:
            return
        s, f = self.screen, self.font
        s.fill((0, 0, 0))
        y = 10
        def line(txt, color=(255,255,255)):
            nonlocal y
            s.blit(f.render(txt, True, color), (10, y)); y += 22

        line(f"Node status : {self.status}", (255,255,0))
        y += 8
        line("RX UNO :", (0,255,0));    line(str(self.rx_uno))
        line("RX MEGA:", (0,255,0));    line(str(self.rx_mega))
        y += 8
        line("TX UNO :", (255,215,0));  line(str(self.tx_uno))
        line("TX MEGA:", (255,215,0));  line(str(self.tx_mega))
        y += 8
        line("/joint_states :", (135,206,250)); line(str(self.topic_vals))
        line("/current_joint_states :",  (173,255, 47));   line(str(self.curr_js))
        pygame.display.flip()

    def stop(self):
        self._stop.set()

# ─────────────── 브리지 노드 ────────────────
def rad2deg(r): return r*180.0/math.pi

class Bridge(Node):
    def __init__(self):
        # super().__init__("berry_serial_bridge")
        super().__init__(
            "berry_serial_bridge",
            # YAML-override 에서 온 모든 키를 자동 선언
            automatically_declare_parameters_from_overrides=True
        )
        self.logger = self.get_logger()
        self.logger.info(f"[DBG0] sys.argv → {sys.argv}")

        # ◼ 포트·보레이트 파라미터
        self.declare_parameter("port_uno",  "/dev/ttyUSB1")
        self.declare_parameter("port_mega", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        prm = self.get_parameters_by_prefix('')

        self.get_logger().info(
            f"Serial ports → UNO:{prm['port_uno'].value}, "
            f"MEGA:{prm['port_mega'].value}, baud:{prm['baud'].value}")


        self.uno  = BoardSerial("UNO",  prm["port_uno"].value,
                                prm["baud"].value,  self._uno_rx)
        self.mega = BoardSerial("MEGA", prm["port_mega"].value,
                                prm["baud"].value,  self._mega_rx)

        # ◼ 조인트 매핑 파라미터 (YAML로 주입)
        # # self.map: Dict[str, Dict] = {}
        # # for name, prm in self._parameters.items():
        # #     if name.startswith("joint_cfg."):
        # #         j = name.split('.',1)[1]
        # #         self.map[j] = prm.value  # dict
        # self.map: Dict[str, Dict] = {}
        # cfg_params = self.get_parameters_by_prefix("joint_cfg")
        # self.logger.info(f"[DBG0] cfg_params → {cfg_params}")
        # for full_name, param in cfg_params.items():
        #     # full_name 예) "joint_cfg.joint1"
        #     j = full_name.split('.', 1)[1]
        #     self.map[j] = param.value

        # ◼ 조인트 매핑 파라미터 (YAML)
        self.map: Dict[str, Dict] = {}

        # ── DEBUG 0 : 전체 로딩된 파라미터 덤프 ──
        for pname, p in self._parameters.items():
            self.get_logger().info(f"[PARAM] {pname} = {p.value}")

        # ── case ① : joint_cfg.* 가 평탄화(flat) → jointX 딕셔너리로 그룹핑 ──name}")
        # ── joint_cfg.* → "joint1.board" 형태이므로 2단계만 처리하면 된다 ──
        cfg_params = self.get_parameters_by_prefix("joint_cfg")
        for key, param in cfg_params.items():
            if '.' in key:
                joint, field = key.split('.', 1)            # 'joint1', 'board'
                self.map.setdefault(joint, {})[field] = param.value
            else:
                # nested-dict 형식이 들어온 경우
                if isinstance(param.value, dict):
                    self.map.setdefault(key, {}).update(param.value)


        # ── case ② : joint_cfg 가 dict 한 방에 들어온 YAML ──
        if not self.map and self.has_parameter("joint_cfg"):
            nested = self.get_parameter("joint_cfg").value          # Dict
            self.get_logger().info(f"[DBG2] nested joint_cfg → {nested}")
            self.map.update(nested)

        if not self.map:
            self.get_logger().info(
                "⚠  joint_cfg 파라미터가 비어 있습니다. YAML 구조, 노드 이름, "
                "launch-파일 parameter 연결을 다시 확인하세요.")

        # ◼ 피드백, 동작 플래그
        self.feedback: Dict[str,float] = {}
        self.homing_ok = {"UNO": False, "MEGA": False}
        self.started   = False

        # ◼ 타이머/토픽
        self.pub = self.create_publisher(JointState,
                                         "/current_joint_states", 10)
        self.timer = self.create_timer(0.1, self._pub_feedback)

        # ◼ 명령 버퍼 + 송신 타이머 (2 Hz)
        self.latest_uno_cmd  = [0.0, 0.0, 0]
        self.latest_mega_cmd = [0.0, 0.0, 0.0]
        self.send_timer = self.create_timer(0.1, self._send_latest_commands)

        self.logger.info("🕒  두 보드 Homing 완료 메시지를 기다리는 중...")
        self.logger.info(f"조인트 매핑 파라미터: {self.map}")

        # ◼ pygame 모니터
        self.ui = PygameDisplay()
        self.ui.status = "Homing..."

    # ── 시리얼 콜백들 ──────────────────────────
    def _uno_rx(self, line:str):
        if "Homing" in line and "완료" in line:
            self.logger.info("UNO Homing 메시지 감지")
            self.homing_ok["UNO"] = True
            self._check_startable()
            self.ui.rx_uno["Z/W"] = "—"                 # 초기화
        if line.startswith("POS"):
            # POS Z:-123.4 W:30 G:1
            try:
                # z = float(line.split()[1].split(':')[1])   # mm
                # w = float(line.split()[2].split(':')[1])   # deg
                # self.feedback["joint1"] = z / -1000.0      # mm→m, sign
                # self.feedback["joint5"] = math.radians(w-90)
                tokens = line.split()
                z_mm   = float(tokens[1].split(':')[1])
                w_deg  = float(tokens[2].split(':')[1])
                g_mode = int(tokens[3].split(':')[1])
                raw_map = {0: z_mm, 1: w_deg, 2: g_mode}
                # invert using joint_cfg parameters
                for joint, cfg in self.map.items():
                    if cfg.get('board') == "UNO":
                        idx = cfg.get('idx')
                        raw = raw_map.get(idx)
                        if raw is None: continue
                        pos = (raw - cfg.get('offset', 0)) / cfg.get('scale', 1)
                        self.feedback[joint] = pos
            except: pass
            # pygame 표시
            self.ui.rx_uno = {"Z_mm": round(z_mm,1), "W_deg": round(w_deg,1)}

    def _mega_rx(self, line:str):
        if "Homing" in line and "done" in line:
            self.logger.info("MEGA Homing 메시지 감지")
            self.homing_ok["MEGA"] = True
            self._check_startable()
            self.ui.rx_mega["X/Y/Z"] = "—"
        if line.startswith("POS"):
            try:
                # _, x, y, z = line.split()
                # d = [float(v.split(':')[1]) for v in (x,y,z)]
                # self.feedback["joint2"] = math.radians(-d[0])
                # self.feedback["joint3"] = math.radians( d[1])
                # self.feedback["joint4"] = math.radians( d[2])
                tokens = line.split()
                x_val = float(tokens[1].split(':')[1])
                y_val = float(tokens[2].split(':')[1])
                z_val = float(tokens[3].split(':')[1])
                raw_map = {0: x_val, 1: y_val, 2: z_val}
                # invert using joint_cfg parameters
                for joint, cfg in self.map.items():
                    if cfg.get('board') == "MEGA":
                        idx = cfg.get('idx')
                        raw = raw_map.get(idx)
                        if raw is None: continue
                        pos = (raw - cfg.get('offset', 0)) / cfg.get('scale', 1)
                        self.feedback[joint] = pos
            except: pass
            # self.ui.rx_mega = {"X": round(d[0],1), "Y": round(d[1],1), "Z": round(d[2],1)}
            self.ui.rx_mega = {"X": round(x_val,1), "Y": round(y_val,1), "Z": round(z_val,1)}

    # ── Homing 확인, 사용자 입력 대기 ────────────
    def _check_startable(self):
        if all(self.homing_ok.values()) and not self.started:
            self.logger.info("✅  Homing 완료!  <Return> 를 누르면 제어를 시작합니다.")
            self.ui.status = "Homing done – press Return"
            threading.Thread(target=self._wait_key, daemon=True).start()

    def _wait_key(self):
        # pygame 창에서 Return 키가 눌릴 때까지 대기
        self.ui.enter_event.wait()
        self.started = True
        # print("🚀  JointState 구독 시작!")
        self.logger.info("🚀  JointState 구독 시작!")
        self.ui.status = "Controlling"
        # 구독은 시작 시점에 생성해야 버퍼를 비우지 않음
        self.create_subscription(JointState, "/joint_states",
                                 self._gui_cb, 10)

    # ── GUI → 명령 ──────────────────────────────
    def _gui_cb(self, msg: JointState):
        if not self.started: return
        name2pos = dict(zip(msg.name, msg.position))
        uno_cmd  = [0.0, 0.0, 0]      # Z_mm W_deg G
        mega_cmd = [0.0, 0.0, 0.0]    # X Y Z deg
        for j,pos in name2pos.items():
            cfg = self.map.get(j);          # {'board':..., 'idx':..., ...}
            if not cfg: continue
            val = pos * cfg['scale'] + cfg['offset']
            if cfg['board']=="UNO":
                uno_cmd[cfg['idx']] = val
            else:
                mega_cmd[cfg['idx']] = val

        # pygame에 표시
        self.ui.tx_uno  = uno_cmd.copy()
        self.ui.tx_mega = mega_cmd.copy()
        self.ui.topic_vals = {n: round(p,3) for n,p in name2pos.items()}

        self.logger.info(f"명령 변환 → UNO {uno_cmd} | MEGA {mega_cmd}")
        # self.uno.send(f"{uno_cmd[0]:.1f} {uno_cmd[1]:.1f} {int(uno_cmd[2])}a\n")
        # self.mega.send(f"{mega_cmd[0]:.1f} {mega_cmd[1]:.1f} {mega_cmd[2]:.1f}a\n")

        # 변환된 명령을 주기송신을 위해 버퍼에 저장
        self.latest_uno_cmd  = uno_cmd.copy()
        self.latest_mega_cmd = mega_cmd.copy()

    # ── 피드백 발행 ──────────────────────────────
    def _pub_feedback(self):
        if not self.feedback: return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = zip(*sorted(self.feedback.items()))
        self.pub.publish(msg)
        self.ui.curr_js = {n: round(p,3) for n,p in zip(msg.name, msg.position)}
        # self.logger.info(f"피드백 퍼블리시: {list(msg.name)}={list(msg.position)}")

    # ── 주기 송신 (2 Hz) ─────────────────────────────
    def _send_latest_commands(self):
        """2 Hz로 최신 버퍼 명령을 보드에 전송"""
        if not self.started: return
        u = self.latest_uno_cmd
        m = self.latest_mega_cmd
        self.logger.info(f"주기 송신 → UNO {u} | MEGA {m}")
        self.uno.send(f"{u[0]:.1f} {u[1]:.1f} {int(u[2])}a\n")
        self.mega.send(f"{m[0]:.1f} {m[1]:.1f} {m[2]:.1f}a\n")
        # UI 갱신
        self.ui.tx_uno  = u.copy()
        self.ui.tx_mega = m.copy()

    # ── 종료 ────────────────────────────────────
    def destroy_node(self):
        self.logger.info("노드 종료: 시리얼 포트 닫는 중")
        self.uno.close(); self.mega.close()
        self.ui.stop()
        super().destroy_node()

# ─────────────── main ────────────────
def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__=="__main__":
    main()
