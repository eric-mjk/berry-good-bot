#!/usr/bin/env python3
"""
uno-mega-serial.py
------------------
● Jetson Orin Nano + Ubuntu 22.04
● UNO, MEGA 두 보드와 동시에 시리얼 통신
● 터미널 한 곳에서 입·출력을 모두 확인

    u-50 0 0a        →  UNO 로 "-50 0 0a\n" 전송
    m90 90 -5a       →  MEGA로 "90 90 -5a\n" 전송
    q                →  스크립트 종료

※ ROS2 연동을 염두에 두고, 보드별 커넥션을 클래스로 캡슐화해 둠
"""
import threading
import sys
import time

import serial            # $ pip install pyserial

# ────────────────────────────────
# ① 보드별 설정 (수정해 주세요)
# ────────────────────────────────
PORT_UNO  = "/dev/ttyUSB1"
PORT_MEGA = "/dev/ttyUSB0"
BAUD      = 115200

# ────────────────────────────────
# ② 시리얼 커넥션 래퍼
# ────────────────────────────────
class BoardSerial:
    def __init__(self, name: str, port: str, baud: int):
        self.name = name          # "UNO" or "MEGA"
        self.port = port
        self.baud = baud
        self.ser  = None
        self._rx_thread = None
        self._stop = threading.Event()

    # 연결 열기
    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        print(f"[{self.name}] 🔌  Opened {self.port} @ {self.baud}")
        # 수신 스레드 시작
        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name=f"{self.name}-RX",
            daemon=True,
        )
        self._rx_thread.start()

    def close(self):
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"[{self.name}] 🔌  Closed")

    # 송신
    def send(self, msg: str):
        if not self.ser or not self.ser.is_open:
            print(f"[{self.name}] ⚠️  Port not open")
            return
        self.ser.write(msg.encode())
        # echo 송신 로그
        print(f"[{self.name}] ▶ {msg!r}")

    # 수신 스레드
    def _rx_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            if self.ser.in_waiting:
                byte = self.ser.read()
                if byte == b'\n':               # 한 줄 완성
                    line = buf.decode(errors='ignore').strip()
                    print(f"[{self.name}] ◀ {line}")
                    buf.clear()
                else:
                    buf.extend(byte)
            else:
                time.sleep(0.01)

# ────────────────────────────────
# ③ 프로그램 진입점
# ────────────────────────────────
def main():
    uno  = BoardSerial("UNO",  PORT_UNO,  BAUD)
    mega = BoardSerial("MEGA", PORT_MEGA, BAUD)

    try:
        uno.open()
        mega.open()

        print("\n📜  입력 형식:  u<공백 포함 원본 명령...>  |  m<공백 포함 원본 명령...>")
        print("    예)  u-120 0 0a   |   m90 45 -5a")
        print("    q  → 종료\n")

        while True:
            try:
                line = input("> ").strip()
            except EOFError:      # Ctrl-D
                break

            if not line:
                continue
            if line.lower() == "q":
                break

            # 판별: 첫 글자
            prefix, payload = line[0].lower(), line[1:]
            if prefix == "u":
                uno.send(payload + "\n")
            elif prefix == "m":
                mega.send(payload + "\n")
            else:
                print("⚠️  접두사(u/m)가 필요합니다.")
    finally:
        uno.close()
        mega.close()

# ────────────────────────────────
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
