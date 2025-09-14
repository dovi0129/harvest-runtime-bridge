from neuromeka import IndyDCP2
import inspect

# 1) 생성자 인자 **순서 주의** (server_ip, robot_name)
indy = IndyDCP2("192.168.0.18", "NRMK-Indy7")

# 2) 연결 먼저 (일부 메서드는 연결 후 바인딩됨)
indy.connect()

# 3) 혹시 동적 SDK 활성화 트리거가 있다면 한번 호출
if hasattr(indy, "active_sdk"):
    try:
        indy.active_sdk()
    except TypeError:
        pass

# 4) 실제로 호출 가능한(public) 메서드 나열 + 시그니처까지
for name, fn in sorted(inspect.getmembers(indy, callable)):
    if name.startswith("_"): 
        continue
    try:
        sig = str(inspect.signature(fn))
    except (ValueError, TypeError):
        sig = "(…)"
    print(f"{name}{sig}")

# 5) 내가 자주 쓰는 후보 이름들 존재 여부 빠르게 체크
cands = [
    "connect","disconnect","get_joint_pos","get_task_pos",
    "joint_move_to","joint_move_by","task_move_to","task_move_by",
    "go_home","go_zero","stop_motion","set_servo","set_brake","set_motion"
]
print({n: hasattr(indy, n) for n in cands})
