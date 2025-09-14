# baek/harvest_runtime/calib/solve_handeye_single.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse, sys
from pathlib import Path
import numpy as np
import cv2
import yaml

# import path
_THIS = Path(__file__).resolve()
_RT = _THIS.parents[2]     # harvest_runtime/
_SYSROOT = _RT.parent      # baek/
if str(_SYSROOT) not in sys.path:
    sys.path.insert(0, str(_SYSROOT))

from harvest_runtime.common.configs import root

def inv_T(T):
    R = T[:3,:3]; t = T[:3,3]
    Ri = R.T; ti = -Ri @ t
    Ti = np.eye(4); Ti[:3,:3]=Ri; Ti[:3,3]=ti
    return Ti

def main():
    ap = argparse.ArgumentParser("Solve hand-eye for SINGLE ArUco samples.npz")
    ap.add_argument("run_dir", help="path under calib/data/runs/ e.g., run-20250914-103000")
    ap.add_argument("--method", choices=["Tsai","Park","Horaud","Andreff","Daniilidis"], default="Tsai")
    args = ap.parse_args()

    base = root() / "calib" / "data" / "runs" / args.run_dir
    npz = np.load(base / "samples.npz", allow_pickle=True)
    T_base_ee = npz["T_base_ee"]   # (N,4,4)
    T_cam_board = npz["T_cam_board"]  # (N,4,4)

    # OpenCV calibrateHandEye inputs: R_gripper2base, t_gripper2base, R_target2cam, t_target2cam
    # Our stored are base->ee and cam->board, so invert accordingly.
    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for Tb2g, Tc2t in zip(T_base_ee, T_cam_board):
        Tg2b = inv_T(Tb2g)          # gripper->base
        Tt2c = inv_T(Tc2t)          # target->camera (board->cam)
        R_g2b.append(Tg2b[:3,:3]); t_g2b.append(Tg2b[:3,3])
        R_t2c.append(Tt2c[:3,:3]); t_t2c.append(Tt2c[:3,3])

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c, method=getattr(cv2, f"CALIB_HAND_EYE_{args.method.upper()}")
    )

    # We want ee_T_cam (gripper->camera). OpenCV returned cam->gripper.
    R_g2c = R_cam2gripper.T
    t_g2c = -R_cam2gripper.T @ t_cam2gripper

    # Save YAML
    out_yaml = root() / "config" / "handeye_T_ee_cam.yaml"
    data = {
        "R": R_g2c.tolist(),
        "t": t_g2c.reshape(3).tolist(),
        "frame_convention": "base_T_ee * ee_T_cam * cam_P",
        "notes": f"Solved from {args.run_dir} by {args.method}"
    }
    with open(out_yaml, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

    print("[RESULT] ee_T_cam saved to", out_yaml)
    print("R =\n", R_g2c)
    print("t =", t_g2c.reshape(3))

if __name__ == "__main__":
    main()
