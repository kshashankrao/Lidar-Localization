import os
import json
import numpy as np
from kitti import KittiDataLoader

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "config.json")

def compute_rpe(gt_poses, est_poses):
    length = min(len(gt_poses), len(est_poses))
    if length < 2:
        return 0.0, 0.0, length
    t_errs, r_errs = [], []
    total_dist = 0.0

    for i in range(1, length):
        dist = np.linalg.norm(gt_poses[i][:3, 3] - gt_poses[i-1][:3, 3])
        total_dist += dist
        if dist < 0.1:
            continue

        T_gt_i_inv = np.linalg.inv(gt_poses[i-1])
        delta_gt = T_gt_i_inv @ gt_poses[i]

        T_est_i_inv = np.linalg.inv(est_poses[i-1])
        delta_est = T_est_i_inv @ est_poses[i]

        error_matrix = np.linalg.inv(delta_gt) @ delta_est
        t_err = np.linalg.norm(error_matrix[:3, 3])
        t_errs.append(t_err / dist)

        trace = np.trace(error_matrix[:3, :3])
        val = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
        r_err = np.arccos(val)
        r_errs.append(r_err / dist)

    avg_t_err_pct = np.mean(t_errs) * 100 if t_errs else 0.0
    avg_r_err_deg_100m = np.degrees(np.mean(r_errs)) * 100 if r_errs else 0.0
    return avg_t_err_pct, avg_r_err_deg_100m, length

def main():
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)

    gt_poses = KittiDataLoader.load_gt_poses(cfg["GT_POSE_PATH"])

    methods = {
        "ICP Only": os.path.join(PROJECT_ROOT, "poses_ICP.txt"),
        "EKF ICP": os.path.join(PROJECT_ROOT, "poses_EKF_ICP.txt"),
        "LOAM": os.path.join(PROJECT_ROOT, "poses_LOAM.txt")
    }

    print("\n" + "="*70)
    print(" CURRENT BENCHMARK COMPARISON AGAINST GROUND TRUTH")
    print("="*70)
    print(f"{'Method':<15} | {'Frames':<8} | {'Trans Error (%)':<18} | {'Rot Error (deg/100m)':<20}")
    print("-" * 70)

    for name, path in methods.items():
        if os.path.exists(path):
            est_poses = KittiDataLoader.load_estimated_poses(path)
            t_err, r_err, count = compute_rpe(gt_poses, est_poses)
            print(f"{name:<15} | {count:<8} | {t_err:<18.3f} | {r_err:<20.3f}")
        else:
            print(f"{name:<15} | {'N/A':<8} | {'N/A':<18} | {'N/A':<20}")
    print("="*70 + "\n")

if __name__ == "__main__":
    main()
