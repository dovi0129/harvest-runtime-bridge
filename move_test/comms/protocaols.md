# 통신 프로토콜 요약

본 문서는 **Indy 7 로봇팔**(IndyDCP 2)과 **OpenMANIPULATOR‑X 그리퍼**(DYNAMIXEL Protocol 2.0)의 통신 규격과 권장 시퀀스를 정리한 것이다.

---

## 1. Indy 7 로봇팔 — IndyDCP 2 (TCP/IP)

### 1.1 연결 정보

| 항목              | 값                                                                        |
| --------------- | ------------------------------------------------------------------------ |
| **기본 IP**       |                                                  |
| **포트**          |                                                  |
| **통신 주기**       | ≤ 100 Hz (10 ms)                                                         |
| **좌표 단위**       | 위치 `x y z` : **mm**<br>자세 `rx ry rz` : **rad** *(ZYX Euler, Tool Frame)* |
| **속도 / 가속도 단위** | mm s⁻¹ / mm s⁻²                                                          |

### 1.2 권장 명령 시퀀스

```text
connect(ip, port=6066)
set_motion_param(speed, acceleration)
move_pose(x, y, z, rx, ry, rz)   # 또는 move_joint(q1…q6)
while not is_idle():              # 상태 폴링
    sleep(0.01)
disconnect()
```

### 1.3 메시지 프레임(struct)

| 오프셋 |    길이 | 필드      | 설명                            |
| --: | ----: | ------- | ----------------------------- |
|   0 |   4 B | "INDY"  | 시그니처                          |
|   4 |   2 B | CMD     | 명령 ID (`0x0031` = MovePose 등) |
|   6 |   4 B | LEN     | Payload 길이 (Little‑Endian)    |
|  10 | *LEN* | Payload | `float32 × N` 파라미터            |

> **모션 완료 확인**은 `is_idle()` / `get_robot_status()` 응답 플래그로 판단한다.

---

## 2. OpenMANIPULATOR‑X 그리퍼 — DYNAMIXEL Protocol 2.0 (UART)

### 2.1 연결 정보

| 항목             | 값                                   |
| ------------     | ----------------------------------- |
| **디바이스**   | Jetson `/dev/ttyTHS1` *(Ex UART 2)* |
| **Baudrate**   | `1 000 000` bps *(기본 1 Mbps)*       |
| **프로토콜 버전**  | `2.0`                               |
| **Motor ID** | `11` *(펌웨어로 변경 가능)*                 |

### 2.2 주요 제어 레지스터 (XM430‑W350‑T)

| 주소(hex) | 이름               | 크기  | 설명                       |
| ------- | ---------------- | --- | ------------------------ |
| `0x40`  | Torque Enable    | 1 B | 0 = Disable / 1 = Enable |
| `0x66`  | Goal Current     | 2 B | 파지 힘 (mA)                |
| `0x68`  | Goal Velocity    | 4 B | 개폐 속도 (rev min⁻¹)        |
| `0x74`  | Goal Position    | 4 B | 개폐 폭 지정(Enc cnt)         |
| `0x84`  | Present Position | 4 B | 현재 위치                    |
| `0x88`  | Present Current  | 2 B | 현재 전류                    |

### 2.3 폭 ↔ 엔코더 매핑 (선형 근사)

*스트로크 20 mm (닫힘) ↔ 75 mm (열림)*

$$
\text{pos}_{cnt}=400+\frac{75-\text{width}_{mm}}{0.275}
$$

| 폭 (mm) |       엔코더 cnt |
| -----: | ------------:      |
|     75 | 400 *(완전 열림)*  |
|     35 |           545     |
|     20 | 600 *(완전 닫힘)* |

> 캘리퍼로 실측 후 회귀 보정하면 ±1 mm 정밀 제어 가능.

### 2.4 메시지 프레임(Byte)

```
0xFF 0xFF 0xFD 0x00  ID  LEN_L LEN_H  INST  PARAM…  CRC_L CRC_H
```

* **ID** : Motor ID
* **INST** : `0x03` = WRITE, `0x02` = READ, …
* **PARAM** : 시작주소 2 B + 데이터

### 2.5 명령 예시 (Python SDK)

```python
# 그리퍼 35 mm 파지
pos_cnt = 545                     # 위 매핑 참조
ph.write4ByteTxRx(port, 11, 0x74, pos_cnt)
# 속도·전류 제한도 필요 시 같이 설정
```

---

## 3. 통신 테스트 스크립트 예시

```bash
python examples/send_arm_pose_example.py        # 로봇팔 이동
python examples/send_gripper_pose_example.py    # 그리퍼 개폐
```

---

### 참고 문서

* Neuromeka **IndyDCP 2** Developer Guide (v2.3)
* ROBOTIS **DYNAMIXEL Protocol 2.0** e‑Manual (XM430‑W350‑T)
