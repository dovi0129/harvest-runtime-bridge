# examples/send_arm_pose_example.py
import logging
from comms.arm_comm import RobotController, INITIAL_TASK_POSE_M_DEG

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    arm = RobotController(ip="192.168.0.18", robot_name="NRMK-Indy7", timeout=0.5)

    print("Task pos:", arm.robot.get_task_pos())
    print("Joint pos:", arm.robot.get_joint_pos())
    
    try:
        arm.connect()

        # ① 홈 확인/이동 → ② 타깃 이동 → ③ 홈으로 복귀
        arm.ensure_home(home_mode="task", timeout=60)  # 기본포즈 서기
        # 상대이동: mm → m 로 변환해서 1mm만 이동, 자세는 그대로(0,0,0)
        arm.robot.task_move_by([1e-3, -1e-3, 1e-3, 0.0, 0.0, 0.0])
        arm.wait_until_motion_complete(timeout=60)
        
        logging.info("Sequence complete.")

    except Exception as e:
        logging.error(f"[ERROR] {e}", exc_info=True)
    finally:
        arm.disconnect()
        logging.info("Robot disconnected.")
