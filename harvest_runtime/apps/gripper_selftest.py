# baek/harvest_runtime/apps/gripper_selftest.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from harvest_runtime.apps.gripper_api import Gripper
from harvest_runtime.common.configs import get_robot

class DummyIndy:
    """U2D2만 사용할 때 로봇 연결이 필요 없으니 더미로 대체"""
    mode = "dummy"
    impl = None

def main():
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    cfg = get_robot().get("gripper", {})
    if (cfg.get("mode") or "").lower() != "u2d2":
        print("[WARN] robot.yaml 의 gripper.mode 가 'u2d2'가 아닙니다. 현재 설정:", cfg.get("mode"))

    grip = Gripper(DummyIndy(), cfg)
    print("[TEST] OPEN")
    grip.open()
    print("[TEST] CLOSE")
    grip.close()
    print("[TEST] OPEN")
    grip.open()

    # 종료 시 포트 정리(선택)
    grip.shutdown()

if __name__ == "__main__":
    main()
