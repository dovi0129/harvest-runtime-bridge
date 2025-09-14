# harvest_runtime/common/schema.py
from __future__ import annotations
from typing import Dict, Any

def _as_float(d: Dict[str, Any], k: str) -> float:
    v = d.get(k, None)
    if v is None:
        raise ValueError(f"missing '{k}'")
    try:
        return float(v)
    except Exception:
        raise ValueError(f"'{k}' must be numeric (got {type(v)})")

def _norm_point(obj: Dict[str, Any]) -> Dict[str, float]:
    return {
        "x": _as_float(obj, "x"),
        "y": _as_float(obj, "y"),
        "z": _as_float(obj, "z"),
    }

def normalize_detection_payload(msg: Dict[str, Any]) -> Dict[str, Any]:
    """
    detection → robot 형식 정규화
    지원:
      A) {"left":{...}, "right":{...}, "angle":...}
      B) {"cmd":"pick","target":{"left":{...},"right":{...},"angle":...}}
    출력: {"left":{x,y,z}, "right":{x,y,z}, "angle":float}
    """
    if "cmd" in msg and msg.get("cmd") == "pick" and isinstance(msg.get("target"), dict):
        payload = msg["target"]
    else:
        payload = msg

    if "left" not in payload or "right" not in payload:
        raise ValueError("payload must contain 'left' and 'right'")
    left = _norm_point(payload["left"])
    right = _norm_point(payload["right"])

    # angle은 선택적으로 허용 (없으면 0.0)
    ang = payload.get("angle", 0.0)
    try:
        angle = float(ang)
    except Exception:
        raise ValueError("'angle' must be numeric")

    return {"left": left, "right": right, "angle": angle}

def is_ack_done_bytes(data: bytes) -> bool:
    """
    로봇 → detection ACK 판별
    - b"ACK_DONE"
    - JSON: {"ack":"done"} 형태도 호환
    """
    if data == b"ACK_DONE":
        return True
    try:
        import json
        j = json.loads(data.decode("utf-8", errors="ignore"))
        return isinstance(j, dict) and j.get("ack") == "done"
    except Exception:
        return False

def make_ack_done_bytes() -> bytes:
    return b"ACK_DONE"
