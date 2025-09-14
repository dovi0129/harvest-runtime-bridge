# examples/send_raw_robot_data.py

import logging
from workspace.baek.comms.arm_comm_old     import RobotController
from comms.gripper_comm import GripperController

logging.basicConfig(level=logging.INFO)

def send_robot_and_grip(
    target: dict,
    arm: RobotController,
    grip: GripperController,
    home_pose: dict
) -> None:
    """
    동작 순서:
    ① 그리퍼 오픈
    ② 로봇팔 타겟 위치로 이동
    ③ 그리퍼 닫기
    ④ 1초 대기
    ⑤ 그리퍼 오픈
    ⑥ 로봇팔을 초기(홈) 위치로 이동
    """

    # 1) 그리퍼 오픈
    grip.open()
    logging.info("→ Gripper opened.")



    # 2) 로봇팔 타겟 위치로 이동
    arm.move_pose(
        target['x'], target['y'], target['z'],
        target['rx'], target['ry'], target['rz'],
        speed=10.0, accel=10.0
    )
    logging.info("→ Arm moved to target.")

    # 3) 그리퍼 닫기
    grip.close()
    logging.info("→ Gripper closed.")

    # 4) 1초 대기
    time.sleep(1.0)

    # 5) 그리퍼 오픈
    grip.open()
    logging.info("→ Gripper opened again.")

    # 6) 로봇팔 초기(홈) 위치로 이동
    arm.move_pose(
        home_pose['x'], home_pose['y'], home_pose['z'],
        home_pose['rx'], home_pose['ry'], home_pose['rz'],
        speed=10.0, accel=10.0
    )
    logging.info("→ Arm returned to home pose.")

if __name__ == "__main__":
    # 타겟 좌표 (예시)
    sample_data = {
        'x': 250.0, 'y': -50.0, 'z': 150.0,
        'rx': 0.0,   'ry': 0.0,   'rz': 90.0
    }
    # 홈(초기) 위치 (예시, 실제 값에 맞게 수정)
    home_pose = {
        'x': 200.0, 'y': 0.0, 'z': 100.0,
        'rx': 0.0,  'ry': 0.0, 'rz': 0.0
    }

    arm  = RobotController(ip="192.168.0.18", port=6066)
    grip = GripperController(device_name="/dev/ttyTHS1", motor_id=11)

    try:
        arm.connect()
        send_robot_and_grip(sample_data, arm, grip, home_pose)
    except Exception as e:
        logging.error(f"Error: {e}")
    finally:
        grip.disconnect()
        arm.disconnect()
        logging.info("All devices disconnected.")