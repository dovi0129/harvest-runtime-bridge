# baek/harvest_runtime/apps/fake_detection_sender.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import json, socket, sys, time, random
from pathlib import Path

# 경로 보정
_THIS = Path(__file__).resolve()
_RT = _THIS.parents[1]   # harvest_runtime/
_SYSROOT = _RT.parent    # baek/
if str(_SYSROOT) not in sys.path:
    sys.path.insert(0, str(_SYSROOT))

from harvest_runtime.common.configs import get_network

def main():
    net = get_network()["robot_bridge"]
    host, tcp_port = net["host"], int(net["tcp_port"])
    ack_port = int(net["ack_port"])

    # TCP 연결(브리지/스모크테스트 수신처)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, tcp_port))
    print(f"[FAKE-SENDER] Connected to {host}:{tcp_port}")

    # UDP ACK 수신 소켓 (브리지가 ack_host:ack_port 로 쏨 → 우리는 ack_port 대기)
    ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ack_sock.bind(("0.0.0.0", ack_port))
    ack_sock.settimeout(5.0)
    print(f"[FAKE-SENDER] Waiting ACK on UDP :{ack_port}")

    try:
        base = [0.30, 0.00, 0.40]  # (m) 대충 카메라 기준 앞쪽
        for i in range(5):         # 5개만 전송 (원하면 늘리세요)
            jx = (random.random()-0.5)*0.06   # ±3 cm
            jy = (random.random()-0.5)*0.04   # ±2 cm
            jz = (random.random()-0.5)*0.02   # ±1 cm

            left  = {"x": round(base[0]+jx, 3), "y": round(base[1]+jy, 3), "z": round(base[2]+jz, 3)}
            right = {"x": round(left["x"]+0.007, 3), "y": round(left["y"]-0.005, 3), "z": round(left["z"]-0.002, 3)}
            msg = {"left": left, "right": right, "angle": round(30 + (random.random()-0.5)*10, 2)}

            line = json.dumps(msg) + "\n"   # 라인 단위 전송(브리지 파서와 호환)
            sock.sendall(line.encode("utf-8"))
            print(f"[FAKE-SENDER] Sent: {msg}")

            # ACK 대기
            try:
                data, addr = ack_sock.recvfrom(128)
                print(f"[FAKE-SENDER] ACK from {addr}: {data!r}")
            except socket.timeout:
                print("[FAKE-SENDER] ACK timeout (continue)")
            time.sleep(0.8)
    except KeyboardInterrupt:
        print("\n[FAKE-SENDER] Interrupted")
    finally:
        try: sock.close()
        except Exception: pass
        try: ack_sock.close()
        except Exception: pass
        print("[FAKE-SENDER] Closed")

if __name__ == "__main__":
    main()
