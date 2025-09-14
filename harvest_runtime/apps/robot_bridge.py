# harvest_runtime/apps/robot_bridge.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import json, math, socket, time, sys
from typing import Iterator, Dict, Any
from pathlib import Path

import numpy as np
import logging
from logging.handlers import RotatingFileHandler

from harvest_runtime.apps.gripper_api import Gripper

# ---- 패키지 경로 보정(직접 실행 시 대비) ------------------------------------
_THIS = Path(__file__).resolve()
_RT = _THIS.parents[1]  # harvest_runtime/
_SYSROOT = _RT.parent   # baek/
if str(_SYSROOT) not in sys.path:
    sys.path.insert(0, str(_SYSROOT))

# ---- 내부 모듈 --------------------------------------------------------------
from harvest_runtime.common.configs import (
    get_network, get_robot, get_handeye, logs_path
)
from harvest_runtime.common.schema import (
    normalize_detection_payload, make_ack_done_bytes
)

# transforms가 없을 경우를 대비해 필수 함수만 안전하게 가져오기
try:
    # [PATCH] rpy_deg_to_R도 함께 가져와 (임시 회전값 사용할 수 있게)
    from harvest_runtime.common.transforms import (
        pose_to_T, T_to_pose, rpy_deg_to_R
    )
except Exception:
    # 최소 대체 구현
    import math as _math

    def rpy_deg_to_R(rpy):
        rx, ry, rz = [_math.radians(a) for a in rpy]
        Rx = np.array([[1,0,0],[0,_math.cos(rx),-_math.sin(rx)],[0,_math.sin(rx),_math.cos(rx)]])
        Ry = np.array([[_math.cos(ry),0,_math.sin(ry)],[0,1,0],[-_math.sin(ry),0,_math.cos(ry)]])
        Rz = np.array([[_math.cos(rz),-_math.sin(rz),0],[_math.sin(rz),_math.cos(rz),0],[0,0,1]])
        return Rz @ Ry @ Rx  # ZYX

    def R_to_rpy_deg(R):
        sy = -R[2,0]; cy = _math.sqrt(max(0.0, 1 - sy*sy))
        if cy > 1e-6:
            rx = _math.atan2(R[2,1], R[2,2]); ry = _math.asin(sy); rz = _math.atan2(R[1,0], R[0,0])
        else:
            rx = _math.atan2(-R[1,2], R[1,1]); ry = _math.asin(sy); rz = 0.0
        return [_math.degrees(rx), _math.degrees(ry), _math.degrees(rz)]

    def pose_to_T(pose_m_deg):  # [x,y,z, rx,ry,rz]
        x,y,z, rx,ry,rz = pose_m_deg
        T = np.eye(4); T[:3,:3] = rpy_deg_to_R([rx,ry,rz]); T[:3,3] = [x,y,z]
        return T

    def T_to_pose(T):
        x,y,z = T[:3,3].tolist()
        # rpy 역변환이 필요하면 위 R_to_rpy_deg 사용
        def _R_to_rpy_deg(R):
            sy = -R[2,0]; cy = _math.sqrt(max(0.0, 1 - sy*sy))
            if cy > 1e-6:
                rx = _math.atan2(R[2,1], R[2,2]); ry = _math.asin(sy); rz = _math.atan2(R[1,0], R[0,0])
            else:
                rx = _math.atan2(-R[1,2], R[1,1]); ry = _math.asin(sy); rz = 0.0
            return [_math.degrees(rx), _math.degrees(ry), _math.degrees(rz)]
        rx,ry,rz = _R_to_rpy_deg(T[:3,:3])
        return [x,y,z, rx,ry,rz]

def h(p3):  # [x,y,z] -> [x,y,z,1]
    return np.array([p3[0], p3[1], p3[2], 1.0], dtype=float)

# -----------------------------------------------------------------------------
# 로깅
# -----------------------------------------------------------------------------
def setup_logging():
    log_path = logs_path("robot_bridge.log")
    handler = RotatingFileHandler(log_path, maxBytes=2_000_000, backupCount=3)
    fmt = logging.Formatter("[%(asctime)s][%(levelname)s] %(message)s")
    handler.setFormatter(fmt)
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    # 중복 핸들러 방지
    if not any(isinstance(h, RotatingFileHandler) for h in root.handlers):
        root.addHandler(handler)
        root.addHandler(logging.StreamHandler())

# -----------------------------------------------------------------------------
# Indy 래퍼 (indy_utils 또는 neuromeka SDK 자동 감지)
# -----------------------------------------------------------------------------
class IndyClient:
    def __init__(self, ip: str, name: str, port: int):
        self.ip, self.name, self.port = ip, name, port
        self.impl = None
        self.mode = None  # "indy_utils" or "neuromeka"

    def connect(self):
        # 1) indy_utils
        try:
            from indy_utils import indydcp_client as ic
            self.impl = ic.IndyDCPClient(self.ip, self.name)
            self.impl.connect()
            self.mode = "indy_utils"
            logging.info(f"[INDY] Connected via indy_utils: {self.ip} {self.name}")
            return
        except Exception as e:
            logging.info(f"[INDY] indy_utils connect failed: {e}")

        # 2) neuromeka IndyDCP2
        try:
            from neuromeka import IndyDCP2
            self.impl = IndyDCP2(self.ip, self.name, str(self.port))
            # 연결 확인
            _ = self.impl.get_task_pos()
            self.mode = "neuromeka"
            logging.info(f"[INDY] Connected via neuromeka IndyDCP2: {self.ip} {self.name}")
            return
        except Exception as e:
            logging.error(f"[INDY] neuromeka connect failed: {e}")
            raise RuntimeError("No Indy client available")

    def disconnect(self):
        try:
            if self.mode == "indy_utils":
                self.impl.disconnect()
        except Exception:
            pass

    def _mm_to_m_if_needed(self, p):
        x,y,z, rx,ry,rz = p
        if max(abs(x),abs(y),abs(z)) > 2.0:  # mm일 가능성 높음
            x,y,z = x/1000.0, y/1000.0, z/1000.0
        return [x,y,z, rx,ry,rz]

    def _m_to_mm_if_needed(self, p):
        # indy_utils/neuromeka 둘 다 m/deg를 잘 받는 경우가 많지만
        # 환경에 따라 mm가 필요한 경우가 있어 안전장치만 둠(필요 시 수정)
        return p

    def get_task_pos(self):
        if self.mode == "indy_utils":
            p = self.impl.get_task_pos()
            return self._mm_to_m_if_needed(p)
        elif self.mode == "neuromeka":
            p = self.impl.get_task_pos()
            return self._mm_to_m_if_needed(p)
        else:
            raise RuntimeError("Indy not connected")

    def task_move_to(self, pose_m_deg):
        if self.mode == "indy_utils":
            self.impl.task_move_to(self._m_to_mm_if_needed(pose_m_deg))
        elif self.mode == "neuromeka":
            self.impl.task_move_to(self._m_to_mm_if_needed(pose_m_deg))
        else:
            raise RuntimeError("Indy not connected")

    def wait_for_move_finish(self):
        if self.mode == "indy_utils":
            self.impl.wait_for_move_finish()
        elif self.mode == "neuromeka":
            self.impl.wait_for_move_finish()
        else:
            raise RuntimeError("Indy not connected")

# -----------------------------------------------------------------------------
# TCP JSON 스트림 파서 (개행/연속 JSON 모두 지원)
# -----------------------------------------------------------------------------
def json_stream(host: str, port: int) -> Iterator[Dict[str, Any]]:
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
                # 개행 단위 우선 처리
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        yield json.loads(line)
                    except Exception:
                        # 라인 단위 파싱 실패 → raw_decode 시도
                        pass
                # 남은 버퍼에 연속 JSON 있을 수 있음
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
        try:
            conn.close()
        except Exception:
            pass
        try:
            srv.close()
        except Exception:
            pass
        logging.info("[TCP] Server closed")

# -----------------------------------------------------------------------------
# 메인 로직
# -----------------------------------------------------------------------------
def main():
    setup_logging()

    net = get_network()["robot_bridge"]
    robot_cfg = get_robot()

    host = net["host"]
    tcp_port = int(net["tcp_port"])
    ack_host = net["ack_host"]
    ack_port = int(net["ack_port"])

    # 모션 파라미터
    keep_ori      = bool(robot_cfg["motion"]["keep_ori"])
    approach_up   = float(robot_cfg["motion"]["approach_up_m"])
    lift_z_base   = float(robot_cfg["motion"]["lift_z_base_m"])
    backoff_ee_z  = float(robot_cfg["motion"]["backoff_ee_z_m"])

    # [PATCH] Hand-Eye 사용/미사용 자동 처리
    # 1) 기본값(I, t_guess) 준비
    R = np.eye(3, dtype=float)
    t = np.array([0.10, 0.00, 0.127], dtype=float)  # 기본 임시값
    use_handeye = True
    # 2) handeye_T_ee_cam.yaml 시도
    try:
        R_loaded, t_loaded = get_handeye(as_numpy=True)
        if R_loaded is not None and t_loaded is not None:
            R, t = R_loaded, t_loaded
            use_handeye = True
        else:
            use_handeye = False
    except Exception as e:
        logging.warning(f"[HAND-EYE] not available. fallback to guess: {e}")
        use_handeye = False
    # 3) 실패 시 robot.yaml의 ee_cam_guess 사용(선택: rpy_deg로 회전도 허용)
    if not use_handeye:
        guess = robot_cfg.get("ee_cam_guess", {})
        t = np.array(guess.get("t_m", t.tolist()), dtype=float)
        rpy = guess.get("rpy_deg", None)
        if rpy is not None:
            try:
                R = rpy_deg_to_R(rpy)
            except Exception:
                R = np.eye(3, dtype=float)

    # 로봇 연결
    indy = IndyClient(
        ip=robot_cfg["indy"]["ip"],
        name=robot_cfg["indy"]["name"],
        port=int(robot_cfg["indy"]["port"])
    )
    indy.connect()

    # 그리퍼 설정(호환성: close_ms/open_ms만 있어도 pulse_ms로 매핑)
    gcfg = dict(robot_cfg.get("gripper", {}))
    if "pulse_ms" not in gcfg:
        try:
            cm = int(gcfg.get("close_ms", 250))
            om = int(gcfg.get("open_ms", 250))
            gcfg["pulse_ms"] = max(cm, om)
        except Exception:
            gcfg["pulse_ms"] = 250
    grip = Gripper(indy, gcfg)

    # ACK 소켓
    ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    logging.info(f"[BRIDGE] TCP {host}:{tcp_port}  ACK→ {ack_host}:{ack_port}  keep_ori={keep_ori}")
    logging.info(f"[HAND-EYE] {'USED' if use_handeye else 'GUESSED'}  R=\n{R}\n t={t.tolist()}")

    # JSON 수신 루프
    for raw in json_stream(host, tcp_port):
        try:
            msg = normalize_detection_payload(raw)
            left, right, angle = msg["left"], msg["right"], msg["angle"]

            # cam_P: 중심점(왼/오 픽킹 포인트 평균)
            cam_P = np.array([
                (left["x"] + right["x"]) / 2.0,
                (left["y"] + right["y"]) / 2.0,
                (left["z"] + right["z"]) / 2.0,
            ], dtype=float)

            # 현재 EE 포즈(베이스)
            curr = indy.get_task_pos()
            T_base_ee = pose_to_T(curr)

            # ee_T_cam
            T_ee_cam = np.eye(4)
            T_ee_cam[:3,:3] = R
            T_ee_cam[:3, 3] = t

            # base 좌표로 투영
            base_P = (T_base_ee @ T_ee_cam @ h(cam_P))[:3]

            # 목표 회전 행렬
            if keep_ori:
                R_target = T_base_ee[:3,:3]
            else:
                # 타겟 바라보도록 z축 정렬
                ee_pos = T_base_ee[:3,3]
                z_axis = base_P - ee_pos
                z_axis /= (np.linalg.norm(z_axis) + 1e-9)
                tmp = np.array([0,0,1.0]) if abs(np.dot(z_axis,[0,0,1.0]))<=0.98 else np.array([0,1.0,0])
                x_axis = np.cross(tmp, z_axis); x_axis /= (np.linalg.norm(x_axis)+1e-9)
                y_axis = np.cross(z_axis, x_axis); y_axis /= (np.linalg.norm(y_axis)+1e-9)
                R_target = np.column_stack([x_axis, y_axis, z_axis])

            # EE +Z(툴 프레임) 방향으로 backoff 후, base +Z로 lift
            ee_plus_z_in_base = R_target[:,2]
            base_P_backed = base_P - ee_plus_z_in_base * backoff_ee_z
            base_P_final  = base_P_backed.copy()
            base_P_final[2] += lift_z_base

            T_target = np.eye(4); T_target[:3,:3] = R_target; T_target[:3,3] = base_P_final
            T_appro  = T_target.copy(); T_appro[2,3] += approach_up

            approach_pose = T_to_pose(T_appro)
            target_pose   = T_to_pose(T_target)

            logging.info(f"[TARGET] cam_P={cam_P} base_P(raw)={base_P}")
            logging.info(f"[MOVE] approach={approach_pose}")
            indy.task_move_to(approach_pose); indy.wait_for_move_finish()

            logging.info(f"[MOVE] target  ={target_pose}")
            indy.task_move_to(target_pose);   indy.wait_for_move_finish()

            # 그리퍼 동작 (실제 DO/더미/U2D2는 Gripper가 모드별 처리)
            # grip.close()
            # time.sleep(0.05)
            # grip.open()

            # 원위치(어프로치로 복귀)
            indy.task_move_to(approach_pose); indy.wait_for_move_finish()

            # ACK
            ack_sock.sendto(make_ack_done_bytes(), (ack_host, ack_port))
            logging.info(f"[ACK] sent to {ack_host}:{ack_port}")

        except Exception as e:
            logging.exception(f"[ERROR] processing message: {e}")
            # 실패 정책 필요 시 NACK/재시도 로직 추가 가능

    try:
        indy.disconnect()
    except Exception:
        pass
    try:
        ack_sock.close()
    except Exception:
        pass
    logging.info("[EXIT] robot bridge terminated")

if __name__ == "__main__":
    main()
