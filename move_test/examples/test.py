import time, logging
from comms.arm_comm import RobotController

logging.basicConfig(level=logging.INFO)
arm = RobotController(ip="192.168.0.18", robot_name="NRMK-Indy7", timeout=0.3)

try:
    arm.connect()

    # 0) 현재 상태 확인(이 버전에선 dict를 바로 반환 → parse 불필요)
    st = arm.robot.get_robot_status()
    print("status:", st)  # {'collision': 1, ...} 등

    # 1) 모션/점유 해제 (있을 수 있는 것만 순차 시도: 실패해도 무시)
    for fn in ["stop_current_program", "stop_motion", "stop_tele_op", "pause_current_program"]:
        if hasattr(arm.robot, fn):
            try: getattr(arm.robot, fn)()
            except: pass

    # 2) 비상/충돌 복구
    if hasattr(arm.robot, "stop_emergency"):
        try: arm.robot.stop_emergency()
        except: pass
    if hasattr(arm.robot, "reset_robot"):   # ★ 충돌/에러 상태 해제의 핵심
        try:
            arm.robot.reset_robot()
            time.sleep(0.2)
        except Exception as e:
            print("reset_robot warn:", e)

    # 3) 제한/감속/충돌 레벨(가능하면 낮춰서 안전하게)
    if hasattr(arm.robot, "set_reduced_mode"):
        try: arm.robot.set_reduced_mode(False)
        except: pass
    if hasattr(arm.robot, "set_reduced_speed_ratio"):
        try: arm.robot.set_reduced_speed_ratio(0.2)   # 아주 천천히
        except: pass
    if hasattr(arm.robot, "set_collision_level"):
        try: arm.robot.set_collision_level(3)
        except: pass

    # 4) 서보/브레이크 (펜던트에서 이미 맞춰져 있으면 실패 로그만 무시)
    if hasattr(arm.robot, "set_brake"):
        try: arm.robot.set_brake(False)
        except Exception as e: print("set_brake warn:", e)
    if hasattr(arm.robot, "set_servo"):
        try: arm.robot.set_servo(True)
        except Exception as e: print("set_servo warn:", e)

    # 5) 충돌 플래그가 내려갔는지 재확인
    st2 = arm.robot.get_robot_status()
    print("status(after reset):", st2)
    if isinstance(st2, dict) and st2.get("collision", 0) != 0:
        print("⚠️ 여전히 collision=1 → 펜던트/ConTY에서 물리적 간섭 해제 후 다시 reset 필요")
    else:
        # 6) 정말 아주 작은 상대이동(1 mm) 테스트: [m, m, m, deg, deg, deg]
        before = arm.robot.get_task_pos()
        print("before:", before)
        arm.robot.task_move_by([1e-3, 0.0, 0.0, 0.0, 0.0, 0.0])  # x로 1 mm
        arm.wait_until_motion_complete(timeout=30)
        after = arm.robot.get_task_pos()
        print("after :", after)
finally:
    arm.disconnect()
