import csv
from workspace.baek.comms.arm_comm_old import RobotController

def read_commands_from_csv(file_path: str):
    """
    CSV 파일의 헤더를 x,y,z,rx,ry,rz 로 가정하고
    모든 명령을 한 번에 읽어 리스트로 반환
    """
    commands = []
    with open(file_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            commands.append((
                float(row['x']),
                float(row['y']),
                float(row['z']),
                float(row['rx']),
                float(row['ry']),
                float(row['rz'])
            ))
    return commands

if __name__ == "__main__":
    # 1) 로봇 컨트롤러 준비
    arm = RobotController(ip="192.168.0.18", port=6066)

    # 2) 파일에서 모든 명령 읽기
    commands = read_commands_from_csv("commands.csv")

    # 3) 순차 실행 (stream_and_act 내부에서 connect/disconnect 처리)
    arm.stream_and_act(commands, speed=1.0, accel=1.0)
