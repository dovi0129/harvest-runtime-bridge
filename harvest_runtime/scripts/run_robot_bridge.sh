#!/usr/bin/env bash
set -euo pipefail

# 스크립트 기준 경로 계산
SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
HR_ROOT="$(realpath "${SCRIPT_DIR}/..")"         # harvest_runtime/
PROJECT_ROOT="$(realpath "${HR_ROOT}/..")"       # baek/ (상위)

# 패키지 경로: 상위 디렉토리를 PYTHONPATH에 추가해야 `harvest_runtime` 임포트 가능
export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"

# (선택) 가상환경 활성화
# source "${PROJECT_ROOT}/.venv/bin/activate" || true

echo "[run_robot_bridge] PYTHONPATH=${PYTHONPATH}"
echo "[run_robot_bridge] launching: python -m harvest_runtime.apps.robot_bridge"
python -m harvest_runtime.apps.robot_bridge
