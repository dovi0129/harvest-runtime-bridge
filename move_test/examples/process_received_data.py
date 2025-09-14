# examples/process_received_data.py
import logging
from vision.calibration import camera_to_robot
from workspace.baek.comms.arm_comm_old     import RobotController
from comms.gripper_comm import GripperController

logging.basicConfig(level=logging.INFO)

def process_and_act(data: dict,
                    arm: RobotController,
                    grip: GripperController) -> None:
    """
    1) 카메라→로봇 좌표 변환
    2) 그리퍼 OPEN
    3) 로봇팔 이동
    4) 그리퍼 CLOSE
    """
    # ── 1) 데이터 언패킹 ───────────────────────────────────
    cx, cy           = data['pixel']         # 픽셀 좌표 (u, v)
    depth_mm         = data['depth_mm']      # 깊이 mm
    angle_deg        = data['angle_deg']     # 과실 축 각도 deg
    logging.info(f"recv pixel=({cx},{cy}) depth={depth_mm} angle={angle_deg}")

    # ── 2) 카메라→로봇 좌표 변환 ──────────────────────────
    x_r, y_r, z_r = camera_to_robot(cx, cy, depth_mm)
    logging.info(f"robot coords  x={x_r:.1f} y={y_r:.1f} z={z_r:.1f}")

    # ── 3) 그리퍼 먼저 OPEN ──────────────────────────────
    grip.open()
    logging.info("gripper OPEN")

    # ── 4) 로봇팔 이동 (move_pose) ────────────────────────
    arm.move_pose(
        x_r, y_r, z_r,
        0.0, 0.0, angle_deg,      # rx, ry, rz (deg)
        speed=100.0,              # mm/s
        accel=1000.0              # mm/s²
    )
    logging.info("arm MOVE 완료")

    # ── 5) 그리퍼 CLOSE ──────────────────────────────────
    grip.close()
    logging.info("gripper CLOSE")


if __name__ == "__main__":
    sample_data = {
        'pixel'    : (382, 290),
        'depth_mm' : 243.0,
        'angle_deg': 47.5
    }

    arm  = RobotController(ip="192.168.0.18", port=6066)
    grip = GripperController(device_name="/dev/ttyUSB0", motor_id=11)  # 실제 포트 확인!

    try:
        arm.connect()
        process_and_act(sample_data, arm, grip)
    except Exception as e:
        logging.error(f"[MAIN] {e}")
    finally:
        grip.disconnect()
        arm.disconnect()
        logging.info("모든 디바이스 disconnect 완료")
