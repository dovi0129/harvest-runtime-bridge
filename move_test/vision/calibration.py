# vision/calibration.py

import numpy as np

R = np.load("extrinsic_R.npy")   # 3×3 회전 행렬
t = np.load("extrinsic_t.npy")   # 3×1 병진 벡터

def camera_to_robot(x_mm: float, y_mm: float, z_mm: float):
    """
    카메라 기준 3D 좌표 (mm) → 로봇 기준 좌표 (mm) 로 변환해 줍니다.
    """
    p_cam = np.array([x_mm, y_mm, z_mm])
    p_rob = R.dot(p_cam) + t
    return p_rob.tolist()  # [X_rob, Y_rob, Z_rob]
