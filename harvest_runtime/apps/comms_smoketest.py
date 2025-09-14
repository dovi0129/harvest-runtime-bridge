# baek/harvest_runtime/apps/comms_smoketest.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import json, socket, sys
from typing import Iterator, Dict, Any
from pathlib import Path

import logging
from logging.handlers import RotatingFileHandler

# ---- 패키지 경로 보정 -------------------------------------------------------
_THIS = Path(__file__).resolve()
_RT = _THIS.parents[1]   # harvest_runtime/
_SYSROOT = _RT.parent    # baek/
if str(_SYSROOT) not in sys.path:
    sys.path.insert(0, str(_SYSROOT))

# ---- 내부 모듈 ---------------------------------------------------------------
from harvest_runtime.common.configs import get_network, logs_path
from harvest_runtime.common.schema import make_ack_done_bytes

# -----------------------------------------------------------------------------
def setup_logging():
    log_path = logs_path("comms_smoketest.log")
    handler = RotatingFileHandler(log_path, maxBytes=1_000_000, backupCount=2)
    fmt = logging.Formatter("[%(asctime)s][%(levelname)s] %(message)s")
    handler.setFormatter(fmt)
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    if not any(isinstance(h, RotatingFileHandler) for h in root.handlers):
        root.addHandler(handler)
        root.addHandler(logging.StreamHandler())

def json_stream(host: str, port: int) -> Iterator[Dict[str, Any]]:
    """detection.py가 보내는 TCP JSON을 라인/연속 JSON 모두 지원해서 수신."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    logging.info(f"[TCP] Listening on {host}:{port}")

    conn, addr = srv.accept()
    conn.settimeout(0.5)
    logging.info(f"[TCP] Connected from {addr}")

    dec = json.JSONDecoder()
    buf = ""
    try:
        while True:
            try:
                chunk = conn.recv(4096)
                if not chunk:
                    logging.info("[TCP] client closed")
                    break
                buf += chunk.decode("utf-8", errors="ignore")
                # 개행 우선
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        yield json.loads(line)
                    except Exception:
                        pass
                # 연속 JSON 처리
                while buf:
                    s = buf.lstrip()
                    if not s:
                        buf = ""
                        break
                    try:
                        obj, idx = dec.raw_decode(s)
                        yield obj
                        buf = s[idx:]
                    except json.JSONDecodeError:
                        break
            except socket.timeout:
                continue
    finally:
        try: conn.close()
        except Exception: pass
        try: srv.close()
        except Exception: pass
        logging.info("[TCP] Server closed")

def main():
    setup_logging()
    net = get_network()["robot_bridge"]
    host, tcp_port = net["host"], int(net["tcp_port"])
    ack_host, ack_port = net["ack_host"], int(net["ack_port"])

    ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    logging.info(f"[SMOKETEST] TCP {host}:{tcp_port}  (ACK→ {ack_host}:{ack_port})")
    logging.info("[SMOKETEST] No robot motion. Just echo & ACK.")

    for msg in json_stream(host, tcp_port):
        logging.info(f"[RECV] {msg}")
        try:
            ack_sock.sendto(make_ack_done_bytes(), (ack_host, ack_port))
            logging.info(f"[ACK] sent to {ack_host}:{ack_port}")
        except Exception as e:
            logging.warning(f"[ACK] send fail: {e}")

    try: ack_sock.close()
    except Exception: pass
    logging.info("[EXIT] comms smoketest terminated")

if __name__ == "__main__":
    main()
