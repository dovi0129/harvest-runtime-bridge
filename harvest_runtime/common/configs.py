# harvest_runtime/common/configs.py
from __future__ import annotations
from pathlib import Path
import yaml
import numpy as np
from typing import Tuple, Dict, Any

# 패키지 루트(= harvest_runtime/)
ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "config"
LOG_DIR = ROOT / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

_cache: Dict[str, Any] = {}

def load_yaml(name: str) -> dict:
    """config/ 아래 YAML 로드 (캐시 미사용)"""
    path = (CONFIG_DIR / name).resolve()
    if not path.exists():
        raise FileNotFoundError(f"[configs] YAML not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def _get_cached(name: str, loader) -> Any:
    if name not in _cache:
        _cache[name] = loader()
    return _cache[name]

def get_network() -> dict:
    """network.yaml (robot_bridge.host/port, ack_host/port)"""
    return _get_cached("network", lambda: load_yaml("network.yaml"))

def get_robot() -> dict:
    """robot.yaml (Indy 접속/모션/그리퍼 설정)"""
    return _get_cached("robot", lambda: load_yaml("robot.yaml"))

def get_handeye(as_numpy: bool = True) -> Tuple[np.ndarray, np.ndarray] | dict:
    """
    handeye_T_ee_cam.yaml
    - as_numpy=True  → (R: np.ndarray (3,3), t: np.ndarray (3,))
    - as_numpy=False → dict 원본 반환
    """
    d = _get_cached("handeye", lambda: load_yaml("handeye_T_ee_cam.yaml"))
    if not as_numpy:
        return d
    R = np.array(d["R"], dtype=float).reshape(3, 3)
    t = np.array(d["t"], dtype=float).reshape(3,)
    return R, t

def logs_path(name: str) -> Path:
    """logs/ 아래 파일 경로 생성"""
    p = (LOG_DIR / name).resolve()
    p.parent.mkdir(parents=True, exist_ok=True)
    return p

def root() -> Path:
    return ROOT

def config_dir() -> Path:
    return CONFIG_DIR
