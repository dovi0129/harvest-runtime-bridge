# examples/send_gripper_pose_example.py

import time
import logging
from comms.gripper_comm import GripperController

if __name__ == "__main__":
    # 로깅 설정 (필요 시)
    logging.basicConfig(level=logging.INFO)

    grip = GripperController(
    device_name="/dev/ttyUSB0",  # U2D2 권장
    baudrate=1_000_000,
    protocol_ver=2.0,
    motor_id=11,
    action_delay=0.2
    )

    try:
        # 그리퍼 열기
        logging.info("Opening gripper …")
        grip.open()

        time.sleep(100.0)

        # 그리퍼 닫기
        logging.info("Closing gripper …")
        grip.close()
        logging.info(f"Position: {grip.get_current_position()}")

        time.sleep(1.0)

    finally:
        # 토크 해제 및 포트 닫기
        grip.disconnect()
        print("Gripper disconnected.")
