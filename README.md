# harvest-runtime-bridge

A communication runtime that connects vision-based strawberry detection to robotic harvesting.  
It handles TCP line-delimited JSON from the detector, dispatches motion/grip commands to the robot stack, and emits UDP ACKs when a pick cycle completes.

> 참고: 본 레포는 영문 중심 문서화이며, 일부 한국어(설명/비고)가 포함됩니다.

---

## Key features
- **Detection → Runtime bridge:** Receives per-fruit JSON (`left`, `right`, `angle`, optional `id`) over **TCP**.
- **Robot integration:** (Pluggable) control for **Indy7** via IndyDCP2; end-effector control for **OpenMANIPULATOR-X** over **UART**.
- **ACK signaling:** Emits **UDP** `{"status":"DONE","id":..., "ts":...}` after each handled message.
- **Schema & validation:** Minimal payload normalization/validation (`angle` or backward-compatible `angle_deg`).

> **Attribution (중요):** The **detection** module in this repository is **adapted from work by Minseo Cho (Hanbat National University)** with adjustments for our runtime interface and data schema.  
> (조민서, 한밭대 — 인터페이스/스키마에 맞게 변경하여 사용)

---

## System components
- **Camera:** Intel RealSense D455 (RGB-D)
- **Compute:** Jetson Orin Nano
- **Arm:** Neuromeka Indy7 (TCP/IP via IndyDCP2)
- **End-effector:** OpenMANIPULATOR-X (UART, TTL)
- **Network:**  
  - Detector → Bridge: **TCP** (line-delimited JSON)  
  - Bridge → ACK consumer: **UDP**  
  - Bridge → Indy7: **TCP/IP** (future hook)  
  - Bridge → Gripper: **UART** (future hook)

---

## Message flow
```mermaid
sequenceDiagram
  autonumber
  participant DET as Detector (DL)
  participant BR as harvest-runtime-bridge
  participant ARM as Indy7 (future)
  participant GRIP as OM-X (future)
  participant ACK as UDP ACK consumer

  DET->>BR: TCP JSON {left,right,angle,id}
  BR->>ARM: (future) move/pick via IndyDCP2
  BR->>GRIP: (future) open/close via UART
  ARM-->>BR: (future) done
  BR->>ACK: UDP {"status":"DONE","id","ts"}
````

### JSON schema (line-delimited over TCP)

**Detector → Bridge**

```json
{"left":{"x":0.325,"y":-0.008,"z":0.390},
 "right":{"x":0.332,"y":-0.013,"z":0.388},
 "angle":12.4,
 "id":"sample-0001"}
```

* `left/right`: 3D points in meters (camera frame; example)
* `angle`: rotation angle in degrees (`angle_deg` also accepted for compatibility)
* `id` (optional): correlation id echoed back in ACK

**Bridge → UDP ACK**

```json
{"status":"DONE","id":"sample-0001","ts":1699950000.123}
```

---

## Repo layout

```
harvest-runtime-bridge/
├─ detection/                 # adapted detector (by Minseo Cho, HN Univ.) + our adapters
├─ harvest_runtime/
│  ├─ apps/
│  │  ├─ robot_bridge.py      # TCP server → (future) robot → UDP ACK
│  │  ├─ comms_smoketest.py   # no-motion smoke test
│  │  └─ fake_detection_sender.py
│  ├─ common/                 # configs.py, schema.py
│  └─ config/                 # *.yaml.example (copy to *.yaml)
├─ docs/                      # MESSAGE_SCHEMA.md, diagrams
├─ scripts/                   # run_* helpers
└─ requirements.txt
```

---

## Quick start

```bash
# 0) Python env (optional)
python -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt

# 1) Prepare configs
cp harvest_runtime/config/network.yaml.example harvest_runtime/config/network.yaml
# edit host/ports if needed

# 2) Smoke test (no robot motion)
bash harvest_runtime/scripts/run_comms_smoketest.sh
# in another terminal
bash harvest_runtime/scripts/send_fake_detection.sh

# 3) Run the real bridge
bash harvest_runtime/scripts/run_robot_bridge.sh
```

---

## Configuration

`harvest_runtime/config/network.yaml`

```yaml
tcp_listen: { host: "127.0.0.1", port: 9999 }
udp_ack:    { host: "127.0.0.1", port: 9998 }
tcp_timeout: 5.0
```

`harvest_runtime/config/robot.yaml`

```yaml
robot:
  ip: "192.168.0.18"
  port: 6066
  name: "NRMK-Indy7"
  timeout: 0.1
```

> 실제 운영에서는 IP/포트/토큰 등 민감 설정을 \*.yaml로 관리하고,
> **commit**에는 `*.yaml.example`만 포함하세요. (`.gitignore`로 보호)

---

## Roadmap

* [x] TCP/UDP messaging + schema normalization
* [x] Fake detector sender & no-motion smoke test
* [ ] Indy7 motion hooks (IndyDCP2) in `robot_bridge.py`
* [ ] OpenMANIPULATOR-X (UART) integration
* [ ] Hand–Eye calibration path (optional module)
* [ ] Stress tests & latency profiling; reliability hardening

---

## License

MIT

### Acknowledgements

* Detection module **adapted** from **Minseo Cho (Hanbat National University)**.
  We appreciate the original implementation and insights. (조민서, 한밭대)
