#!/usr/bin/env bash
set -euo pipefail

# 경로 설정
SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
HR_ROOT="$(realpath "${SCRIPT_DIR}/..")"
PROJECT_ROOT="$(realpath "${HR_ROOT}/..")"
export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"

python - <<'PY'
import socket, sys, time
from harvest_runtime.common.configs import get_network

net = get_network()
ack_host = net["robot_bridge"]["ack_host"]
ack_port = int(net["robot_bridge"]["ack_port"])

print(f"[fake-ack] sending ACK_DONE to {ack_host}:{ack_port}")
print("Press ENTER to send one ACK, or Ctrl+C to exit.")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    while True:
        _ = sys.stdin.readline()
        sock.sendto(b"ACK_DONE", (ack_host, ack_port))
        print("[fake-ack] ACK_DONE sent.")
except KeyboardInterrupt:
    pass
finally:
    sock.close()
PY
