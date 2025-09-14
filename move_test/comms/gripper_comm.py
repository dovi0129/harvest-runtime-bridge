# comms/gripper_comm.py

import time
import logging
from typing import Optional
from dynamixel_sdk import PortHandler, PacketHandler

logger = logging.getLogger(__name__)

class GripperController:
    # Control table addresses for XM430-W350 
    ADDR_TORQUE_ENABLE    = 64
    ADDR_GOAL_POSITION    = 116
    ADDR_PRESENT_POSITION = 132

    OPEN_POSITION  = 400   # 열림: 조절 필요
    CLOSE_POSITION = 600   # 닫힘: 조절 필요

    TORQUE_ENABLE  = 1
    TORQUE_DISABLE = 0

    def __init__(
        self,
        device_name: str = "/dev/ttyUSB0",
        baudrate: int = 1_000_000,
        protocol_ver: float = 2.0,
        motor_id: int = 15,
        action_delay: float = 0.2
    ):
        self.device_name  = device_name
        self.baudrate     = baudrate
        self.protocol_ver = float(protocol_ver)  # ← 먼저 세팅!
        self.motor_id     = int(motor_id)
        self.delay        = float(action_delay)

        self.portHandler   = PortHandler(self.device_name)
        self.packetHandler = PacketHandler(self.protocol_ver)  # ← 그다음 사용
        self.connected     = False

        if not self.portHandler.openPort():
            raise RuntimeError(f"Failed to open port {self.device_name}")
        if not self.portHandler.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set baudrate {self.baudrate}")

        # 토크 ON (write 계열은 반환값 2개)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE
        )
        if dxl_comm_result != 0 or dxl_error != 0:
            raise RuntimeError(f"Torque enable failed: comm={dxl_comm_result}, err={dxl_error}")

        self.connected = True

    def _write_position(self, pos: int) -> None:
        if not isinstance(pos, int):
            raise ValueError(f"Position must be int, got {type(pos)}")
        if not self.connected:
            raise RuntimeError("GripperController: not connected")

        # --- 반환값 언팩 수정 (2개) ---
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.motor_id, self.ADDR_GOAL_POSITION, pos
        )
        if dxl_comm_result != 0 or dxl_error != 0:
            raise RuntimeError(f"Write position failed: comm={dxl_comm_result}, err={dxl_error}")

        time.sleep(self.delay)

    def get_current_position(self) -> int:
        if not self.connected:
            raise RuntimeError("GripperController: not connected")

        # --- read는 3개 반환(값, 통신, 에러) ---
        val, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, self.motor_id, self.ADDR_PRESENT_POSITION
        )
        if dxl_comm_result != 0 or dxl_error != 0:
            raise RuntimeError(f"Read position failed: comm={dxl_comm_result}, err={dxl_error}")
        return val

    def disconnect(self) -> None:
        if self.connected:
            try:
                # --- 반환값 언팩 수정 (2개) ---
                self.packetHandler.write1ByteTxRx(
                    self.portHandler, self.motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE
                )
            finally:
                self.portHandler.closePort()
                self.connected = False