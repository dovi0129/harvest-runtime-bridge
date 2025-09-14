# examples/nudge_open.py
import time
import logging
from comms.gripper_comm import GripperController

logging.basicConfig(level=logging.INFO)

# 사용자 설정
DEVICE   = "/dev/ttyUSB0"   # U2D2면 보통 이거. Jetson UART 쓰면 "/dev/ttyTHS1"
MOTOR_ID = 11
STEP     = 20               # 한 번에 움직일 틱 수 (작게 시작: 10~30 권장)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

if __name__ == "__main__":
    grip = GripperController(
        device_name=DEVICE,
        baudrate=1_000_000,
        protocol_ver=2.0,
        motor_id=MOTOR_ID,
        action_delay=0.2
    )
    try:
        cur = grip.get_current_position()
        open_hint = grip.OPEN_POSITION  # 기본 400 
        # open_hint 쪽으로 한 스텝만 이동
        direction = 1 if open_hint > cur else -1
        target = clamp(cur + direction * STEP, 0, 4095)

        logging.info(f"Gripper nudge towards OPEN: cur={cur} -> target={target} (step={STEP})")
        # 내부 메서드지만 간단히 사용 (원하면 public move_to 추가해도 됨)
        grip._write_position(target)

        time.sleep(0.5)
        now = grip.get_current_position()
        logging.info(f"Now at: {now}")

    finally:
        grip.disconnect()
        print("Gripper disconnected.")
