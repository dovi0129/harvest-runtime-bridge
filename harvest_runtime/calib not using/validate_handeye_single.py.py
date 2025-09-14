# -*- coding: utf-8 -*-
"""
Hand-Eye(ee_T_cam) 검증 스크립트 (단일 ArUco 수집 결과용)
- 입력: calib/data/runs/<run_dir>/samples.npz  (T_base_ee, T_cam_board, K, dist, size)
- 입력: config/handeye_T_ee_cam.yaml           (R, t)
- 출력: AX=XB 잔차 통계, base_T_board 일관성(고정물 가정) 통계
사용 예:
  python -m calib.validate_handeye_single run-20250914-103000
"""
from __future__ import annotations

import argparse, sys
from pathlib import Path
import numpy as np

# --- 모듈 경로 보정 (직접 실행 대비) ---
_THIS = Path(__file__).resolve()
_RT = _THIS.parents[2]     # harvest_runtime/
_SYSROOT = _RT.parent      # baek/
if str(_SYSROOT) not in sys.path:
    sys.path.insert(0, str(_SYSROOT))

from harvest_runtime.common.configs import root, get_handeye

# ----------------- SE(3) 유틸 -----------------
def inv_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]; t = T[:3, 3]
    Ri = R.T
    ti = -Ri @ t
    Ti = np.eye(4); Ti[:3, :3] = Ri; Ti[:3, 3] = ti
    return Ti

def rot_angle_deg(R: np.ndarray) -> float:
    # 회전오차 각도(deg): acos((trace(R)-1)/2)
    v = (np.trace(R) - 1.0) / 2.0
    v = np.clip(v, -1.0, 1.0)
    return float(np.degrees(np.arccos(v)))

def pose_err_stats(deg_list, mm_list, title):
    if not deg_list:
        print(f"[WARN] {title}: 비교쌍이 없습니다.")
        return
    d = np.array(deg_list, float)
    m = np.array(mm_list, float)
    print(f"\n[{title}] (pairs={len(d)})")
    print(f"  rot_err(deg): mean={d.mean():.3f}, std={d.std():.3f}, min={d.min():.3f}, max={d.max():.3f}")
    print(f"  trans_err(mm): mean={m.mean():.3f}, std={m.std():.3f}, min={m.min():.3f}, max={m.max():.3f}")

# ----------------- 메인 -----------------
def main():
    ap = argparse.ArgumentParser("Validate Hand-Eye result with single ArUco run.")
    ap.add_argument("run_dir", help="calib/data/runs/ 아래 폴더명 (예: run-20250914-103000)")
    ap.add_argument("--pair-mode", choices=["sequential", "all"], default="sequential",
                    help="잔차 평가 쌍 선택: sequential(i,i+1) 또는 all(i<j)")
    args = ap.parse_args()

    base = root() / "calib" / "data" / "runs" / args.run_dir
    npz_path = base / "samples.npz"
    if not npz_path.exists():
        raise FileNotFoundError(f"samples.npz not found: {npz_path}")

    data = np.load(npz_path, allow_pickle=True)
    T_base_ee: np.ndarray = data["T_base_ee"]     # (N,4,4) base<-ee
    T_cam_board: np.ndarray = data["T_cam_board"] # (N,4,4) cam<-board
    N = T_base_ee.shape[0]
    print(f"[LOAD] samples: N={N}  from {npz_path}")

    if N < 2:
        print("[WARN] 샘플이 2개 미만이면 잔차 계산이 의미가 없습니다.")

    # Hand-Eye (ee_T_cam)
    R, t = get_handeye(as_numpy=True)
    T_ee_cam = np.eye(4); T_ee_cam[:3,:3] = R; T_ee_cam[:3,3] = t
    print("[LOAD] ee_T_cam from config/handeye_T_ee_cam.yaml")
    print("R=\n", R)
    print("t=", t)

    # ─────────────────────────────────────────────────────────────
    # 1) AX = XB 잔차 (A: gripper motion, B: camera motion, X: ee_T_cam)
    #    여기서 gripper은 'ee', camera는 'cam', target은 'board'
    #    A_ij = inv(T_base_ee_i) @ T_base_ee_j
    #    B_ij = inv(T_cam_board_i)^-1?  -> camera motion은 target->camera로 쓰는 게 보통이나
    #    우리 저장은 cam<-board (Tc2t)이므로 target->camera = inv(T_cam_board).
    #    따라서 B_ij = inv(Tt2c_i) @ Tt2c_j with Tt2c = inv(T_cam_board)
    # ─────────────────────────────────────────────────────────────
    pair_indices = []
    if args.pair_mode == "sequential":
        pair_indices = [(i, i+1) for i in range(N-1)]
    else:  # all
        for i in range(N-1):
            for j in range(i+1, N):
                pair_indices.append((i, j))

    rot_err_deg_list, trans_err_mm_list = [], []
    for (i, j) in pair_indices:
        A_ij = inv_T(T_base_ee[i]) @ T_base_ee[j]

        Tt2c_i = inv_T(T_cam_board[i])   # target->camera
        Tt2c_j = inv_T(T_cam_board[j])
        B_ij = inv_T(Tt2c_i) @ Tt2c_j    # = (cam->target)_i ^-1  @ (target->camera)_j ? (정의에 유의)
        # 더 직관적인 형태로 쓰면: B_ij = inv(Tt2c_i) @ Tt2c_j

        # 잔차:  (A X) (X B)^-1  → I 에 가까워야 함
        Left  = A_ij @ T_ee_cam
        Right = T_ee_cam @ B_ij
        Delta = Left @ inv_T(Right)

        r_err = rot_angle_deg(Delta[:3,:3])
        t_err = np.linalg.norm(Delta[:3,3]) * 1000.0  # mm
        rot_err_deg_list.append(r_err)
        trans_err_mm_list.append(t_err)

    pose_err_stats(rot_err_deg_list, trans_err_mm_list, "AX=XB residual")

    # ─────────────────────────────────────────────────────────────
    # 2) base_T_board 일관성 검사 (보드가 base에 고정되어 있다고 가정)
    #    base_T_cam_i = base_T_ee_i @ ee_T_cam
    #    base_T_board_i_pred = base_T_cam_i @ cam_T_board_i  (우리는 cam<-board 저장이므로 cam_T_board = T_cam_board)
    #    모든 i에서 base_T_board_i_pred가 거의 동일해야 함 → 0번째를 기준으로 비교
    # ─────────────────────────────────────────────────────────────
    base_T_board_list = []
    for i in range(N):
        base_T_cam = T_base_ee[i] @ T_ee_cam
        base_T_board_pred = base_T_cam @ T_cam_board[i]
        base_T_board_list.append(base_T_board_pred)

    if N >= 2:
        ref = base_T_board_list[0]
        degs, mms = [], []
        for i in range(1, N):
            D = inv_T(ref) @ base_T_board_list[i]
            degs.append(rot_angle_deg(D[:3,:3]))
            mms.append(np.linalg.norm(D[:3,3]) * 1000.0)
        pose_err_stats(degs, mms, "base_T_board consistency (vs first sample)")
    else:
        print("\n[INFO] base_T_board consistency: 샘플이 1개 뿐입니다.")

    # 참고: 필요하면 결과를 파일로 저장
    report = {
        "pairs": len(pair_indices),
        "ax_xb_rot_deg_mean": float(np.mean(rot_err_deg_list)) if rot_err_deg_list else None,
        "ax_xb_trans_mm_mean": float(np.mean(trans_err_mm_list)) if trans_err_mm_list else None,
    }
    out = base / "validate_report.json"
    try:
        import json
        with open(out, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"\n[SAVED] report → {out}")
    except Exception:
        pass

if __name__ == "__main__":
    main()
