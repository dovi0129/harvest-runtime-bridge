# comms/arm_comm.py
"""
Indy 7 로봇팔 제어 클래스 (TCP/IP - Neuromeka IndyDCP 2)
───────────────────────────────────────────────────────
• 라이브러리 : neuromeka.IndyDCP2
• 포   트 : 6066 (IndyDCP 2 서버 기본)
• 좌   표 : 위치 mm, 자세 rad  (ZYX Euler)
"""

import time
import math
import logging
import numpy as np
from typing import Optional, Iterable, Tuple
from neuromeka import IndyDCP2
import time

logger = logging.getLogger(__name__)


class RobotController:

    def __init__(
        self,
        ip: str = "192.168.0.18",
        port: int = 6066,
        robot_name: str = "NRMK-Indy7",
        timeout: float = 0.1,
    ):
        self.ip = ip
        self.port = port
        self.robot_name = robot_name
        self.timeout = timeout

        self.robot = IndyDCP2(self.ip, self.robot_name, str(self.port))

        # self.robot = IndyDCP2(self.ip, str(self.port), self.robot_name)

        self.connected: bool = False

    
    def is_at_home(self, home_joint=None, tol=1.0):
        """
        현재 joint 위치가 홈 joint와 tol 이내면 True
        home_joint: 홈 포지션 joint 값(list)
        tol: 허용 오차(도)
        """
        if home_joint is None:
            # 실제 홈 위치 joint 값(teach pendant에서 확인)
            home_joint = [0.6641709769149318, -0.18659262194383724, 0.6428025171041961, 70.04510698794209, 114.17416901151486, 71.6619850679869]
        cur_joint = self.robot.get_joint_pos()
        diff = np.abs(np.array(cur_joint) - np.array(home_joint))
        return np.all(diff < tol)

    def go_home(self, home_joint=None, tol=1.0):
        """홈 포지션으로 이동"""
        if not self.connected:
            raise RuntimeError("RobotController: call connect() first")
        if home_joint is None:
            home_joint = [0.6641709769149318, -0.18659262194383724, 0.6428025171041961, 70.04510698794209, 114.17416901151486, 71.6619850679869]
        if self.is_at_home(home_joint, tol):
            logger.info("[Indy7] 이미 홈 위치에 있음 → 이동·대기 스킵")
            return
        # self.robot.reset_robot()  # 홈 이동 전 리셋이 반드시 필요한 경우만 사용

        self.task_move_to([0.6641709769149318, -0.18659262194383724, 0.6428025171041961, 70.04510698794209, 114.17416901151486, 71.6619850679869])
        self.wait_until_motion_complete()
        logger.info("[Indy7] 홈 위치 이동 및 완료 대기")


    # ────────────────────────────────
    # 연결 / 해제
    # ────────────────────────────────
    def connect(self) -> None:
        """로봇 컨트롤러와 연결"""
        if self.connected:
            return
        self.robot.connect()
        time.sleep(self.timeout)
        self.connected = True
        logger.info(f"[Indy7] connected  ({self.ip}:{self.port})")

    def disconnect(self) -> None:
        """연결 해제"""
        if self.connected:
            self.robot.disconnect()
            self.connected = False
            logger.info("[Indy7] disconnected")

    def move_pose(
        self,
        x: float,
        y: float,
        z: float,
        rx_deg: float,
        ry_deg: float,
        rz_deg: float,
        wait: bool = True,
        wait_timeout: float = 10.0,
    ) -> None:
        """
        Tool Frame Pose 이동 (Task-Space)

        Parameters
        ----------
        x, y, z : mm
        rx_deg, ry_deg, rz_deg : deg  (입력값은 deg → 내부 rad 변환)
        speed : mm/s
        accel : mm/s²
        wait : bool
            True 면 모션 완료까지 블로킹
        """
        if not self.connected:
            raise RuntimeError("RobotController: call connect() first")

        # deg → rad
        rx = math.radians(rx_deg)
        ry = math.radians(ry_deg)
        rz = math.radians(rz_deg)

        # 목표 자세 전송
        pose = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        self.robot.task_move_to(pose)

        logger.info(
            f"[Indy7] move_pose → ({x:.1f},{y:.1f},{z:.1f}, "
            f"{rx_deg:.1f}°, {ry_deg:.1f}°, {rz_deg:.1f}°) "
        )

    def wait_until_motion_complete(self, timeout=30.0, poll=0.1):
        """로봇이 움직임을 모두 마칠 때까지 대기"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if hasattr(self.robot, 'is_idle'):
                if self.robot.is_idle():
                    return True
            time.sleep(poll)
        raise TimeoutError("로봇 모션 완료 대기 중 타임아웃 발생")


    def stream_and_act(
        self,
        command_generator: Iterable[Tuple[float, float, float, float, float, float]],
    ) -> None:
        """
        반복적으로 좌표·자세 명령을 받아 순차 실행

        Parameters
        ----------
        command_generator : iterable of (x, y, z, rx_deg, ry_deg, rz_deg)
        speed, accel : mm/s, mm/s² (모든 명령에 동일하게 적용)
        """
        self.connect()
        # self.joint_move_by([0, 0, 0, 0, 90, -35])

        try:
            # for x, y, z, rx_deg, ry_deg, rz_deg in command_generator:
            #     self.joint_move_by([x, y, z, rx_deg, ry_deg, rz_deg])
            self.joint_move_by([0, 0, 0, 0, 80, -35]) # 임시 이동 명령
        finally:
            self.disconnect()
