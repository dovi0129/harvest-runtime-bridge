# comms/arm_comm.py
"""
Indy 7 로봇팔 제어 래퍼 (Neuromeka IndyDCP2)
───────────────────────────────────────────────────────
• 라이브러리 : neuromeka.IndyDCP2
• 포   트 : 6066 (서버 고정)
• 단   위 : 외부 입력 → x,y,z = mm / rx,ry,rz = deg
            DCP2 호출 직전 → [m, deg] 로 변환
"""

import time
import logging
from typing import Optional, Iterable, Tuple, List
from neuromeka import IndyDCP2

logger = logging.getLogger(__name__)

# ── 프로젝트 표준 포즈 (m,deg) ─────────────────────────────────
INITIAL_TASK_POSE_M_DEG: List[float] = [
    0.6641709769149318, -0.18659262194383724, 0.6428025171041961,
    70.04510698794209, 114.17416901151486, 71.6619850679869
]


class RobotController:
    def __init__(
        self,
        ip: str = "192.168.0.18",
        robot_name: str = "NRMK-Indy7",
        timeout: float = 0.1,
        default_task_vel_level: Optional[int] = 5,   # 1~10
        default_joint_vel_level: Optional[int] = 5,  # 1~10
    ):
        self.ip = ip
        self.robot_name = robot_name
        self.timeout = timeout
        self.default_task_vel_level = default_task_vel_level
        self.default_joint_vel_level = default_joint_vel_level

        # DCP2 생성자: (server_ip, robot_name, robot_version='')
        self.robot = IndyDCP2(self.ip, self.robot_name)
        self.connected: bool = False

    # ────────────────────────────────
    # 연결 / 해제
    # ────────────────────────────────
    def connect(self) -> None:
        """컨트롤러 연결 및 기본 상태 설정"""
        if self.connected:
            return
        self.robot.connect()
        time.sleep(self.timeout)

        # (선택) 통신 타임아웃
        if hasattr(self.robot, "set_timeout_sec"):
            try:
                self.robot.set_timeout_sec(3.0)
            except Exception:
                pass

        # Servo ON / Brake OFF
        if hasattr(self.robot, "get_servo_state"):
            try:
                onoff, brakes = self.robot.get_servo_state()  # 구현에 따라 형태 다를 수 있음
                need_servo = not all(onoff) if isinstance(onoff, (list, tuple)) else True
                need_brake = any(brakes)   if isinstance(brakes, (list, tuple)) else True
            except Exception:
                need_servo = need_brake = True
        else:
            need_servo = need_brake = True
        
        n = self.robot.get_servo_num() if hasattr(self.robot, "get_servo_num") else 6
        on, off = [1]*n, [0]*n
        if need_brake and hasattr(self.robot, "set_brake"):
            try: self.robot.set_brake(off)
            except: pass
        if need_servo and hasattr(self.robot, "set_servo"):
            try: self.robot.set_servo(on)
            except: pass

        # 속도 레벨 기본값
        def _lv(v: int) -> int: return max(1, min(10, int(v)))
        if self.default_task_vel_level is not None and hasattr(self.robot, "set_task_vel_level"):
            self.robot.set_task_vel_level(_lv(self.default_task_vel_level))
        if self.default_joint_vel_level is not None and hasattr(self.robot, "set_joint_vel_level"):
            self.robot.set_joint_vel_level(_lv(self.default_joint_vel_level))

        self.connected = True
        logger.info(f"[Indy7] connected  ({self.ip})  servo=ON, brake=OFF")

    def disconnect(self) -> None:
        """연결 해제"""
        if not self.connected:
            return
        try:
            if hasattr(self.robot, "stop_motion"):
                self.robot.stop_motion()
        except Exception:
            pass
        self.robot.disconnect()
        self.connected = False
        logger.info("[Indy7] disconnected")

    # ────────────────────────────────
    # 상태/도움 함수
    # ────────────────────────────────
    def wait_until_motion_complete(self, timeout: float = 30.0, poll: float = 0.1) -> bool:
        """
        모션 완료 대기:
        ① wait_for_move_finish() 있으면 사용
        ② 폴백: get_robot_status()/parse_robot_status() 폴링
        """
        t0 = time.time()
        if hasattr(self.robot, "wait_for_move_finish"):
            while time.time() - t0 < timeout:
                try:
                    self.robot.wait_for_move_finish()
                    return True
                except Exception:
                    time.sleep(poll)
            raise TimeoutError("모션 완료 대기 타임아웃")

        while time.time() - t0 < timeout:
            try:
                if hasattr(self.robot, "get_robot_status"):
                    st = self.robot.get_robot_status()
                    if hasattr(self.robot, "parse_robot_status"):
                        parsed = self.robot.parse_robot_status(st)
                        moving = bool(
                            parsed.get("is_busy", False) or
                            parsed.get("is_moving", False) or
                            parsed.get("busy", False)
                        )
                        if not moving:
                            return True
            except Exception:
                pass
            time.sleep(poll)
        raise TimeoutError("모션 완료 대기 타임아웃")

    def is_at_task_pose(
        self,
        target_m_deg: Iterable[float],
        tol_xyz_mm: float = 1.0,
        tol_ang_deg: float = 1.0
    ) -> bool:
        """현재 태스크 포즈가 target과 허용오차 이내인지 검사. target=[m,m,m,deg,deg,deg]"""
        cur = self.robot.get_task_pos()  # [m,m,m,deg,deg,deg]
        dx_mm = abs((cur[0] - target_m_deg[0]) * 1000.0)
        dy_mm = abs((cur[1] - target_m_deg[1]) * 1000.0)
        dz_mm = abs((cur[2] - target_m_deg[2]) * 1000.0)
        du = abs(cur[3] - target_m_deg[3])
        dv = abs(cur[4] - target_m_deg[4])
        dw = abs(cur[5] - target_m_deg[5])
        return (dx_mm <= tol_xyz_mm and dy_mm <= tol_xyz_mm and dz_mm <= tol_xyz_mm
                and du <= tol_ang_deg and dv <= tol_ang_deg and dw <= tol_ang_deg)

    # ────────────────────────────────
    # 동작 명령
    # ────────────────────────────────
    def go_home(self, wait: bool = True, timeout: float = 30.0) -> None:
        """컨트롤러에 저장된 홈(Teach Home)으로 이동"""
        if not self.connected:
            raise RuntimeError("call connect() first")
        if hasattr(self.robot, "go_home"):
            self.robot.go_home()
        else:
            raise NotImplementedError("go_home API 필요")
        logger.info("[Indy7] go_home() called")
        if wait:
            self.wait_until_motion_complete(timeout=timeout)

    def go_initial_task_pose(
        self,
        pose_m_deg: Optional[Iterable[float]] = None,
        wait: bool = True,
        timeout: float = 30.0
    ) -> None:
        """사용자 정의 '기본 태스크 포즈'(m,deg)로 이동"""
        if not self.connected:
            raise RuntimeError("call connect() first")
        if pose_m_deg is None:
            pose_m_deg = INITIAL_TASK_POSE_M_DEG
        self.robot.task_move_to([
            float(pose_m_deg[0]), float(pose_m_deg[1]), float(pose_m_deg[2]),
            float(pose_m_deg[3]), float(pose_m_deg[4]), float(pose_m_deg[5])
        ])
        logger.info("[Indy7] go_initial_task_pose() called")
        if wait:
            self.wait_until_motion_complete(timeout=timeout)

    def ensure_home(
        self,
        home_mode: str = "task",      # "task" or "joint"
        tol_xyz_mm: float = 1.0,
        tol_ang_deg: float = 1.0,
        timeout: float = 60.0,
        pose_m_deg: Optional[Iterable[float]] = None,
    ) -> None:
        """
        현재가 홈인지 확인하고, 아니면 홈으로 이동.
        home_mode="task" → 초기 태스크 포즈 기준(is_at_task_pose)
        home_mode="joint"→ 컨트롤러 go_home() 호출
        """
        if home_mode == "task":
            target = pose_m_deg if pose_m_deg is not None else INITIAL_TASK_POSE_M_DEG
            if not self.is_at_task_pose(target, tol_xyz_mm, tol_ang_deg):
                self.go_initial_task_pose(target, wait=True, timeout=timeout)
        elif home_mode == "joint":
            self.go_home(wait=True, timeout=timeout)
        else:
            raise ValueError("home_mode must be 'task' or 'joint'.")

    def move_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        rx_deg: float,
        ry_deg: float,
        rz_deg: float,
        wait: bool = True,
        timeout: float = 30.0,
        vel_level: Optional[int] = None,
    ) -> None:
        """
        Task-space 이동. 입력(mm/deg) → 내부 [m,deg]
        """
        if not self.connected:
            raise RuntimeError("call connect() first")

        if vel_level is not None and hasattr(self.robot, "set_task_vel_level"):
            self.robot.set_task_vel_level(max(1, min(10, int(vel_level))))

        pose_m_deg = [
            float(x_mm) / 1000.0,
            float(y_mm) / 1000.0,
            float(z_mm) / 1000.0,
            float(rx_deg),
            float(ry_deg),
            float(rz_deg),
        ]
        self.robot.task_move_to(pose_m_deg)
        logger.info(
            f"[Indy7] task_move_to → "
            f"({x_mm:.1f},{y_mm:.1f},{z_mm:.1f} mm, "
            f"{rx_deg:.1f}°, {ry_deg:.1f}°, {rz_deg:.1f}°)"
        )
        if wait:
            self.wait_until_motion_complete(timeout=timeout)

    def move_from_home_and_back(
        self,
        target_mm_deg: Tuple[float, float, float, float, float, float],
        home_mode: str = "task",
        tol_xyz_mm: float = 1.0,
        tol_ang_deg: float = 1.0,
        timeout: float = 60.0,
        vel_level: Optional[int] = 5,
        pose_m_deg: Optional[Iterable[float]] = None,
    ) -> None:
        """
        ① 홈 확인/이동 → ② 타깃 이동 → ③ 홈으로 복귀
        target_mm_deg: (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
        """
        if not self.connected:
            self.connect()

        # ① 홈 보장
        self.ensure_home(home_mode=home_mode,
                         tol_xyz_mm=tol_xyz_mm, tol_ang_deg=tol_ang_deg,
                         timeout=timeout, pose_m_deg=pose_m_deg)

        # ② 타깃 이동
        x, y, z, rx, ry, rz = target_mm_deg
        self.move_pose(x, y, z, rx, ry, rz, wait=True, timeout=timeout, vel_level=vel_level)

        # ③ 홈 복귀
        if home_mode == "task":
            self.go_initial_task_pose(pose_m_deg or INITIAL_TASK_POSE_M_DEG, wait=True, timeout=timeout)
        else:
            self.go_home(wait=True, timeout=timeout)

    # with 문 지원
    def __enter__(self):
        self.connect()
        return self
    def __exit__(self, exc_type, exc, tb):
        self.disconnect()
