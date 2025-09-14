# baek/harvest_runtime/apps/gripper_api.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import time
import logging

class Gripper:
    """
    그리퍼 제어 래퍼.
    - mode="u2d2": Robotis U2D2 + Dynamixel SDK (USB 시리얼/TTL 3핀)
      * dx_model: "x_series"(XM/XL430 등, Prot2.0, 4바이트 목표위치),
                  "xl320"(Prot2.0, 2바이트), "ax12a"(Prot1.0, 2바이트)
    - mode="do":   Indy 디지털 출력 기반 open/close 펄스
    - mode="dummy":실기 없이 대기만
    """
    def __init__(self, indy_client, cfg: dict):
        self.indy = indy_client
        self.cfg = cfg or {}
        self.mode = (self.cfg.get("mode") or "do").lower()

        # U2D2 / Dynamixel
        self.dx_PortHandler = None
        self.dx = None
        self.dx_protocol = 2.0
        self.dx_id = 1
        self.addr_TORQUE_ENABLE = None
        self.addr_GOAL_POSITION = None
        self.addr_MOVING_SPEED = None
        self.len_GOAL_POSITION = None
        self.len_MOVING_SPEED = None
        self.open_pos = None
        self.close_pos = None
        self.moving_speed = None
        self.settle_sec = float(self.cfg.get("settle_sec", 0.25))

        if self.mode == "u2d2":
            self._init_u2d2()
        elif self.mode == "do":
            logging.info("[GRIPPER] mode=DO (digital output)")
        else:
            self.mode = "dummy"
            logging.info("[GRIPPER] mode=DUMMY")

    # ------------------------- U2D2 (DYNAMIXEL) -------------------------
    def _init_u2d2(self):
        try:
            from dynamixel_sdk import PortHandler, PacketHandler
        except Exception as e:
            raise RuntimeError(
                "dynamixel-sdk 미설치. requirements.txt에 'dynamixel-sdk' 추가 후 설치하세요."
            ) from e

        port = self.cfg.get("u2d2_port", "/dev/ttyUSB0")
        baud = int(self.cfg.get("u2d2_baudrate", 1000000))
        self.dx_protocol = float(self.cfg.get("dx_protocol", 2.0))  # 1.0 or 2.0
        model = (self.cfg.get("dx_model") or "x_series").lower()

        self.dx_PortHandler = PortHandler(port)
        if not self.dx_PortHandler.openPort():
            raise RuntimeError(f"U2D2 포트 오픈 실패: {port}")
        if not self.dx_PortHandler.setBaudRate(baud):
            raise RuntimeError(f"U2D2 보레이트 설정 실패: {baud}")
        self.dx = PacketHandler(self.dx_protocol)
        logging.info(f"[GRIPPER] mode=U2D2 ({port}, {baud}bps, proto={self.dx_protocol})")

        # 모델 프로파일(주소/길이)
        if model == "x_series":
            # X-series (XM/XL430 등, Protocol 2.0)
            self.addr_TORQUE_ENABLE  = int(self.cfg.get("addr_TORQUE_ENABLE", 64))
            self.addr_GOAL_POSITION  = int(self.cfg.get("addr_GOAL_POSITION", 116))  # 4 bytes
            self.addr_MOVING_SPEED   = int(self.cfg.get("addr_MOVING_SPEED", 112))   # Profile Velocity (4 bytes)
            self.len_GOAL_POSITION   = int(self.cfg.get("len_GOAL_POSITION", 4))
            self.len_MOVING_SPEED    = 4
            # 동작 파라미터(범위 0~4095)
            self.open_pos     = int(self.cfg.get("open_pos", 2000))
            self.close_pos    = int(self.cfg.get("close_pos", 2900))
            self.moving_speed = int(self.cfg.get("moving_speed", 100))
        elif model == "xl320":
            # XL-320 (Protocol 2.0, 2바이트 위치)
            self.addr_TORQUE_ENABLE  = int(self.cfg.get("addr_TORQUE_ENABLE", 24))
            self.addr_GOAL_POSITION  = int(self.cfg.get("addr_GOAL_POSITION", 30))   # 2 bytes
            self.addr_MOVING_SPEED   = int(self.cfg.get("addr_MOVING_SPEED", 32))    # 2 bytes
            self.len_GOAL_POSITION   = int(self.cfg.get("len_GOAL_POSITION", 2))
            self.len_MOVING_SPEED    = 2
            # 동작 파라미터(범위 0~1023)
            self.open_pos     = int(self.cfg.get("open_pos", 300))
            self.close_pos    = int(self.cfg.get("close_pos", 700))
            self.moving_speed = int(self.cfg.get("moving_speed", 200))
        elif model == "ax12a":
            # AX-12A (Protocol 1.0, 2바이트 위치)
            self.addr_TORQUE_ENABLE  = int(self.cfg.get("addr_TORQUE_ENABLE", 24))
            self.addr_GOAL_POSITION  = int(self.cfg.get("addr_GOAL_POSITION", 30))   # 2 bytes
            self.addr_MOVING_SPEED   = int(self.cfg.get("addr_MOVING_SPEED", 32))    # 2 bytes
            self.len_GOAL_POSITION   = int(self.cfg.get("len_GOAL_POSITION", 2))
            self.len_MOVING_SPEED    = 2
            # 동작 파라미터(범위 0~1023)
            self.open_pos     = int(self.cfg.get("open_pos", 300))
            self.close_pos    = int(self.cfg.get("close_pos", 700))
            self.moving_speed = int(self.cfg.get("moving_speed", 200))
        else:
            raise ValueError("dx_model 은 'x_series' | 'xl320' | 'ax12a' 중 하나여야 합니다.")

        # 토크 ON & 속도 설정
        self._dx_write1(self.addr_TORQUE_ENABLE, 1)
        if self.len_MOVING_SPEED == 4:
            self._dx_write4(self.addr_MOVING_SPEED, self.moving_speed)
        else:
            self._dx_write2(self.addr_MOVING_SPEED, self.moving_speed)

        # ID
        self.dx_id = int(self.cfg.get("dx_id", 1))

    # --- Dynamixel write helpers ---
    def _dx_write1(self, addr: int, val: int):
        try:
            self.dx.write1ByteTxRx(self.dx_PortHandler, self.dx_id, addr, int(val))
        except Exception as e:
            logging.warning(f"[GRIPPER] write1 addr={addr} val={val} err={e}")

    def _dx_write2(self, addr: int, val: int):
        try:
            self.dx.write2ByteTxRx(self.dx_PortHandler, self.dx_id, addr, int(val))
        except Exception as e:
            logging.warning(f"[GRIPPER] write2 addr={addr} val={val} err={e}")

    def _dx_write4(self, addr: int, val: int):
        try:
            self.dx.write4ByteTxRx(self.dx_PortHandler, self.dx_id, addr, int(val))
        except Exception as e:
            logging.warning(f"[GRIPPER] write4 addr={addr} val={val} err={e}")

    def _dx_goal_position(self, pos: int):
        if self.len_GOAL_POSITION == 4:
            pos = max(0, min(int(pos), 4095))
            self._dx_write4(self.addr_GOAL_POSITION, pos)
        else:
            pos = max(0, min(int(pos), 1023))
            self._dx_write2(self.addr_GOAL_POSITION, pos)

    # --------------------------- DO (Indy) ---------------------------
    def _set_do(self, ch: int, val: bool):
        try:
            if getattr(self.indy, "mode", None) == "indy_utils":
                self.indy.impl.set_do(int(ch), bool(val))
            elif getattr(self.indy, "mode", None) == "neuromeka":
                self.indy.impl.set_do(int(ch), bool(val))
            else:
                raise RuntimeError("Indy not connected")
        except Exception as e:
            logging.warning(f"[GRIPPER] set_do({ch},{val}) failed: {e}")

    # --------------------------- API ---------------------------
    def open(self):
        if self.mode == "u2d2":
            logging.info(f"[GRIPPER] U2D2 OPEN → pos={self.open_pos}")
            self._dx_goal_position(self.open_pos)
            time.sleep(self.settle_sec)
        elif self.mode == "do":
            pulse = int(self.cfg.get("pulse_ms", 250))
            ch = int(self.cfg.get("do_open", 0))
            logging.info(f"[GRIPPER] DO OPEN on DO{ch} ({pulse}ms pulse)")
            self._set_do(ch, True);  time.sleep(pulse/1000.0);  self._set_do(ch, False)
        else:
            logging.info(f"[GRIPPER] DUMMY OPEN ({self.cfg.get('pulse_ms', 250)}ms)")
            time.sleep(int(self.cfg.get("pulse_ms", 250))/1000.0)

    def close(self):
        if self.mode == "u2d2":
            logging.info(f"[GRIPPER] U2D2 CLOSE → pos={self.close_pos}")
            self._dx_goal_position(self.close_pos)
            time.sleep(self.settle_sec)
        elif self.mode == "do":
            pulse = int(self.cfg.get("pulse_ms", 250))
            ch = int(self.cfg.get("do_close", 1))
            logging.info(f"[GRIPPER] DO CLOSE on DO{ch} ({pulse}ms pulse)")
            self._set_do(ch, True);  time.sleep(pulse/1000.0);  self._set_do(ch, False)
        else:
            logging.info(f"[GRIPPER] DUMMY CLOSE ({self.cfg.get('pulse_ms', 250)}ms)")
            time.sleep(int(self.cfg.get("pulse_ms", 250))/1000.0)

    def shutdown(self):
        """선택: 프로그램 종료 시 호출하면 U2D2 포트를 정리합니다."""
        try:
            if self.mode == "u2d2" and self.dx_PortHandler is not None:
                try:
                    # 토크 OFF(필요 시)
                    if self.addr_TORQUE_ENABLE is not None:
                        self._dx_write1(self.addr_TORQUE_ENABLE, 0)
                except Exception:
                    pass
                self.dx_PortHandler.closePort()
        except Exception:
            pass
