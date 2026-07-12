"""
evaluation.py -- Compare ICP, EKF_GPS, and EKF_IMU performance.

Usage:
    python evaluation.py
"""

import json
import os
import subprocess
import tempfile
import sys
import time
import numpy as np

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH  = os.path.join(PROJECT_ROOT, "config", "config.json")
BINARY_PATH  = os.path.join(PROJECT_ROOT, "build", "bin", "lidar_localization")

sys.path.insert(0, SCRIPT_DIR)
from kitti import KittiDataLoader
from optuna_tuner import Validator, run_binary

def load_base_config() -> dict:
    with open(CONFIG_PATH, "r") as f:
        return json.load(f)

def run_eval(method: str, base_cfg: dict, gt_poses: list) -> tuple:
    print(f"Evaluating {method}...")
    cfg = base_cfg.copy()
    cfg["LOCALIZATION_METHOD"] = method
    cfg["TOTAL_FRAMES_TO_PROCESS"] = base_cfg.get("TOTAL_FRAMES_TO_PROCESS", 1000)
    
    tmp_poses = tempfile.mktemp(suffix=".txt", dir=PROJECT_ROOT)
    cfg["POSES_OUTPUT_PATH"] = tmp_poses

    tmp_cfg = tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False, dir=PROJECT_ROOT)
    json.dump(cfg, tmp_cfg, indent=4)
    tmp_cfg.close()

    t0 = time.time()
    success = run_binary(BINARY_PATH, tmp_cfg.name, timeout=1800)
    elapsed = time.time() - t0

    if not success or not os.path.exists(tmp_poses):
        print(f"  [ERROR] Failed to run {method}")
        return float('inf'), float('inf'), elapsed

    est_poses = KittiDataLoader.load_estimated_poses(tmp_poses)
    val = Validator(gt_poses, est_poses)
    t_err, r_err = val.rpe()

    # Clean up
    os.remove(tmp_cfg.name)
    out_pose_path = os.path.join(PROJECT_ROOT, f"poses_{method}.txt")
    if os.path.exists(out_pose_path):
        os.remove(out_pose_path)
    os.rename(tmp_poses, out_pose_path)

    print(f"  [OK] {method} | t_err: {t_err:.3f}% | r_err: {r_err:.3f} deg/100m | time: {elapsed:.1f}s")
    return t_err, r_err, elapsed

def main():
    base_cfg = load_base_config()
    print("Loading GT poses...")
    gt_poses = KittiDataLoader.load_gt_poses(base_cfg["GT_POSE_PATH"])
    print(f"Loaded {len(gt_poses)} GT frames.")

    methods = ["ICP", "EKF_GPS", "EKF_ICP", "LOAM"]
    results = {}

    for method in methods:
        t_err, r_err, elapsed = run_eval(method, base_cfg, gt_poses)
        results[method] = {"t_err": t_err, "r_err": r_err, "time": elapsed}

    print("\n" + "="*60)
    print(" EVALUATION COMPARISON (1000 frames)")
    print("="*60)
    print(f"{'Method':<12} | {'Trans Error (%)':<15} | {'Rot Error (deg/100m)':<20} | {'Time (s)':<10}")
    print("-" * 60)
    for method in methods:
        t_err = results[method]["t_err"]
        r_err = results[method]["r_err"]
        t     = results[method]["time"]
        print(f"{method:<12} | {t_err:<15.3f} | {r_err:<20.3f} | {t:<10.1f}")
    print("="*60)

if __name__ == "__main__":
    main()
