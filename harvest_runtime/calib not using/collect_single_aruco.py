# baek/harvest_runtime/calib/collect_single_aruco.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse, time, math, sys
from pathlib import Path
from typing import List, Optional

import numpy as np
import cv2
import pyrealsense2 as rs

# import path (direct run support)
_THIS = Path(__file__).resolve()
_RT = _THIS.parents[2]     # harvest_runtime/
_SYSROOT = _RT.parent      # baek/
if str(_SYSROOT) not in sys.path:
    sys.path.insert(0, str(_SYSROOT))

from harvest_runtime.common.transforms import pose_to_T
from harvest_runtime.common.configs import root

# ---------------- Indy wrapper ----------------
class IndyClient:
    def __init__(self, ip: str, name: str, port: int):
        self.ip, self.name, self.port = ip, name, port
        self.impl, self.mode = None, None

    def connect(self):
        try:
            from indy_utils import indydcp_client as ic
            self.impl = ic.IndyDCPClient(self.ip, self.name); self.impl.connect()
            self.mode = "indy_utils"; print(f"[INDY] connected via indy_utils: {self.ip} {self.name}"); return
        except Exception as e:
            print("[INDY] indy_utils failed:", e)
        try:
            from neuromeka import IndyDCP2
            self.impl = IndyDCP2(self.ip, self.name, str(self.port))
            _ = self.impl.get_task_pos()
            self.mode = "neuromeka"; print(f"[INDY] connected via neuromeka: {self.ip} {self.name}"); return
        except Exception as e:
            print("[INDY] neuromeka failed:", e); raise RuntimeError("No Indy client available")

    def disconnect(self):
        try:
            if self.mode == "indy_utils":
                self.impl.disconnect()
        except Exception:
            pass

    def _mm_to_m_if_needed(self, p):
        x,y,z, rx,ry,rz = p
        if max(abs(x),abs(y),abs(z)) > 2.0:  # likely mm
            x,y,z = x/1000.0, y/1000.0, z/1000.0
        return [x,y,z, rx,ry,rz]

    def get_task_pos(self):
        if self.mode == "indy_utils":
            return self._mm_to_m_if_needed(self.impl.get_task_pos())
        elif self.mode == "neuromeka":
            return self._mm_to_m_if_needed(self.impl.get_task_pos())
        else:
            raise RuntimeError("Indy not connected")

# -------------- ArUco single utilities --------------
ARUCO_DICTS = {
    "4X4_50":   cv2.aruco.DICT_4X4_50,
    "5X5_100":  cv2.aruco.DICT_5X5_100,
    "6X6_100":  cv2.aruco.DICT_6X6_100,
    "7X7_100":  cv2.aruco.DICT_7X7_100,
    "APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

def estimate_pose_single(gray, K, dist, dict_name: str, marker_len_m: float) -> Optional[np.ndarray]:
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
    det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return None
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_len_m, K, dist)
    rvec = rvecs[0].reshape(3,1)
    tvec = tvecs[0].reshape(3,1)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=float); T[:3,:3]=R; T[:3,3]=tvec.flatten()
    return T  # camera->board

# -------------- RealSense intrinsics --------------
def rs_intrinsics(p: rs.pipeline_profile):
    prof = p.get_stream(rs.stream.color).as_video_stream_profile()
    intr = prof.get_intrinsics()
    K = np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]], dtype=np.float32)
    dist = np.array(list(intr.coeffs[:5]), dtype=np.float32)
    size = (intr.width, intr.height)
    return K, dist, size

# ---------------------------- main ----------------------------
def main():
    ap = argparse.ArgumentParser("Collect (T_base_ee, T_cam_board) samples with SINGLE ArUco.")
    ap.add_argument("--robot-ip", default="192.168.0.18")
    ap.add_argument("--robot-name", default="NRMK-Indy7")
    ap.add_argument("--robot-port", type=int, default=6066)
    ap.add_argument("--aruco-dict", choices=list(ARUCO_DICTS.keys()), default="5X5_100")
    ap.add_argument("--marker-length", type=float, default=0.04, help="marker side length (meter)")
    ap.add_argument("--out", default=None, help="run dir name under calib/data/runs/")
    args = ap.parse_args()

    runs_root = root() / "calib" / "data" / "runs"
    runs_root.mkdir(parents=True, exist_ok=True)
    run_dir = runs_root / (args.out or time.strftime("run-%Y%m%d-%H%M%S"))
    img_dir = run_dir / "images"; img_dir.mkdir(parents=True, exist_ok=True)
    print("[OUT]", run_dir)

    # Robot
    indy = IndyClient(args.robot_ip, args.robot_name, int(args.robot_port)); indy.connect()

    # Camera
    pipe = rs.pipeline(); cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    prof = pipe.start(cfg)
    K, dist, size = rs_intrinsics(prof)
    print(f"[CAM] size={size}, fx={K[0,0]:.1f}, fy={K[1,1]:.1f}, cx={K[0,2]:.1f}, cy={K[1,2]:.1f}")

    T_base_ee_list: List[np.ndarray] = []
    T_cam_board_list: List[np.ndarray] = []
    idx = 0
    print("\n[INFO] Controls →  C:Capture,  U:Undo,  Q:Quit\n")
    try:
        while True:
            frames = pipe.wait_for_frames()
            col = frames.get_color_frame()
            if not col: continue
            img = np.asanyarray(col.get_data())
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            T_cam_board = estimate_pose_single(gray, K, dist, args.aruco_dict, args.marker_length)

            vis = img.copy()
            if T_cam_board is not None:
                # axis draw (0.05m)
                R = T_cam_board[:3,:3]; t = T_cam_board[:3,3].reshape(3,1)
                rvec, _ = cv2.Rodrigues(R)
                axis = np.float32([[0.05,0,0],[0,0.05,0],[0,0,0.05]]).reshape(-1,3)
                proj, _ = cv2.projectPoints(axis, rvec, t, K, dist)
                c = (int(K[0,2]), int(K[1,2]))
                cv2.line(vis, c, tuple(proj[0,0].astype(int)), (0,0,255), 2)
                cv2.line(vis, c, tuple(proj[1,0].astype(int)), (0,255,0), 2)
                cv2.line(vis, c, tuple(proj[2,0].astype(int)), (255,0,0), 2)
                cv2.putText(vis, "BOARD OK", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,200,0), 2)
            else:
                cv2.putText(vis, "BOARD NOT FOUND", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)

            cv2.putText(vis, f"samples: {len(T_base_ee_list)}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            cv2.imshow("collect_single_aruco", vis)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord('q'), ord('Q')): break
            if key in (ord('u'), ord('U')):
                if T_base_ee_list:
                    T_base_ee_list.pop(); T_cam_board_list.pop()
                    print("[UNDO] last removed. count=", len(T_base_ee_list))
                continue
            if key in (ord('c'), ord('C')):
                if T_cam_board is None:
                    print("[CAPTURE] board not detected → skip"); continue
                p = indy.get_task_pos()
                T_base_ee = pose_to_T(p)     # m/deg → 4x4
                T_base_ee_list.append(T_base_ee)
                T_cam_board_list.append(T_cam_board)
                fn = img_dir / f"cap_{idx:03d}.png"
                cv2.imwrite(str(fn), img)
                idx += 1
                print(f"[CAPTURE] #{idx:03d} (count={len(T_base_ee_list)})  saved:", fn.name)

        # save
        if len(T_base_ee_list) < 4:
            print("[WARN] samples are few; try to capture 10~20 for accuracy.")
        run_dir.mkdir(parents=True, exist_ok=True)
        np.savez(run_dir / "samples.npz",
                 T_base_ee=np.stack(T_base_ee_list, 0),
                 T_cam_board=np.stack(T_cam_board_list, 0),
                 K=K, dist=dist, size=np.array(size, np.int32),
                 meta=np.array(["single", args.aruco_dict, args.marker_length], dtype=object))
        with open(run_dir / "meta.txt", "w", encoding="utf-8") as f:
            f.write(f"created: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"board: single {args.aruco_dict} len={args.marker_length} m\n")
            f.write(f"samples: {len(T_base_ee_list)}\n")
        print("[DONE] saved to", run_dir)
    finally:
        try: rs.pipeline().stop()
        except Exception: pass
        cv2.destroyAllWindows()
        try: indy.disconnect()
        except Exception: pass

if __name__ == "__main__":
    main()
