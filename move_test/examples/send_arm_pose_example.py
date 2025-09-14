# examples/send_arm_pose_example.py
import logging
from comms.arm_comm import RobotController, INITIAL_TASK_POSE_M_DEG

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    arm = RobotController(ip="192.168.0.18", robot_name="NRMK-Indy7", timeout=0.5)

    try:
        arm.connect()

        # ① 홈 확인/이동 → ② 타깃 이동 → ③ 홈으로 복귀
        target = (1.0, -1.0, 1.0, 0.0, 0.0, 90.0)  # (mm, deg)
        arm.move_from_home_and_back(
            target_mm_deg=target,
            home_mode="task",                 # "task" → INITIAL_TASK_POSE_M_DEG 기준
            tol_xyz_mm=1.0, tol_ang_deg=1.0,
            timeout=60.0,
            vel_level=5,
            pose_m_deg=INITIAL_TASK_POSE_M_DEG  # 필요 시 다른 기본포즈로 교체 가능
        )

        logging.info("Sequence complete.")
    except Exception as e:
        logging.error(f"[ERROR] {e}", exc_info=True)
    finally:
        arm.disconnect()
        logging.info("Robot disconnected.")
