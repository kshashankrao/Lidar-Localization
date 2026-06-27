"""
optuna_tuner.py -- Hyperparameter tuning for ICP LiDAR Localization using Optuna.

Usage:
    python optuna_tuner.py [--n-trials 50] [--frames 200] [--binary ./build/bin/lidar_localization]

Strategy:
    Each trial:
      1. Proposes 6 ICP hyperparameter values.
      2. Writes a temporary config.json.
      3. Runs the C++ binary as a subprocess.
      4. Computes KITTI RPE (translation error % + lambda * rotation error deg/100m).
      5. Returns the combined metric as the objective to minimise.
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import optuna
from optuna.visualization import (
    plot_optimization_history,
    plot_param_importances,
    plot_parallel_coordinate,
)

# Path to project root (one level above this script)
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH  = os.path.join(PROJECT_ROOT, "config", "config.json")

def load_base_config() -> dict:
    with open(CONFIG_PATH, "r") as f:
        return json.load(f)

sys.path.insert(0, SCRIPT_DIR)
from kitti import KittiDataLoader


class Validator:
    def __init__(self, gt_poses: list, est_poses: list):
        length = min(len(gt_poses), len(est_poses))
        self.gt  = gt_poses[:length]
        self.est = est_poses[:length]
        self.length = length

    def rpe(self) -> tuple:
        """Returns (avg_translation_err_pct, avg_rotation_err_deg_per_100m)."""
        t_errs, r_errs = [], []
        for i in range(1, self.length):
            dist = np.linalg.norm(self.gt[i][:3, 3] - self.gt[i - 1][:3, 3])
            if dist < 0.1:
                continue
            delta_gt  = np.linalg.inv(self.gt[i - 1])  @ self.gt[i]
            delta_est = np.linalg.inv(self.est[i - 1]) @ self.est[i]
            err       = np.linalg.inv(delta_gt) @ delta_est

            t_err = np.linalg.norm(err[:3, 3])
            t_errs.append(t_err / dist)

            trace = np.trace(err[:3, :3])
            val   = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
            r_errs.append(np.arccos(val) / dist)

        avg_t = np.mean(t_errs) * 100               if t_errs else float("inf")
        avg_r = np.degrees(np.mean(r_errs)) * 100   if r_errs else float("inf")
        return avg_t, avg_r


def write_trial_config(base_cfg: dict, trial_params: dict, frames: int, out_poses: str) -> str:
    cfg = base_cfg.copy()
    cfg.update(trial_params)
    cfg["TOTAL_FRAMES_TO_PROCESS"] = frames
    cfg["POSES_OUTPUT_PATH"]       = out_poses

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", delete=False, dir=PROJECT_ROOT
    )
    json.dump(cfg, tmp, indent=4)
    tmp.close()
    return tmp.name


def run_binary(binary_path: str, config_file: str, timeout: int = 3600) -> bool:
    """Run the ICP binary with the given config. Returns True on success."""
    try:
        result = subprocess.run(
            [binary_path, config_file],
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=os.path.dirname(binary_path),
        )
        if result.returncode != 0:
            print(f"[WARN] Binary returned code {result.returncode}")
            print(result.stderr[-500:])
            return False
        return True
    except subprocess.TimeoutExpired:
        print("[WARN] Binary timed out.")
        return False
    except FileNotFoundError:
        raise RuntimeError(f"Binary not found: {binary_path}")


def make_objective(base_cfg, binary_path, gt_poses, frames, rotation_weight):
    def objective(trial: optuna.Trial) -> float:
        # 1. Suggest hyperparameters
        params = {
            "VOXEL_LEAF_SIZE":             trial.suggest_float("voxel_leaf_size",            0.1,  0.5),
            "MAX_CORRESPONDENCE_DISTANCE": trial.suggest_float("max_correspondence_distance", 0.5,  5.0),
            "MAX_ITERATIONS":              trial.suggest_int(  "max_iterations",              10,  100),
            "NORMAL_K_SEARCH":             trial.suggest_int(  "normal_k_search",             10,   50),
            "TRANSFORMATION_EPSILON":      trial.suggest_float("transformation_epsilon",       1e-12, 1e-6, log=True),
            "EUCLIDEAN_FITNESS_EPSILON":   trial.suggest_float("euclidean_fitness_epsilon",    1e-5,  0.1,  log=True),
            
            # EKF Noise Parameters
            "EKF_PROCESS_NOISE":           trial.suggest_float("ekf_process_noise",            1e-4, 1.0, log=True),
            "EKF_GPS_NOISE":               trial.suggest_float("ekf_gps_noise",                1e-3, 5.0, log=True),
            "EKF_ICP_NOISE":               trial.suggest_float("ekf_icp_noise",                1e-3, 1.0, log=True),
        }

        # 2. Write temp config and output poses path
        tmp_poses = tempfile.mktemp(suffix=".txt", dir=PROJECT_ROOT)
        tmp_cfg   = write_trial_config(base_cfg, params, frames, tmp_poses)

        try:
            # 3. Run C++ binary
            t0      = time.time()
            success = run_binary(binary_path, tmp_cfg)
            elapsed = time.time() - t0
            trial.set_user_attr("runtime_s", round(elapsed, 1))

            if not success or not os.path.exists(tmp_poses):
                return float("inf")

            # 4. Load estimated poses
            est_poses = KittiDataLoader.load_estimated_poses(tmp_poses)
            if len(est_poses) < 5:
                return float("inf")

            # 5. Compute RPE
            val = Validator(gt_poses[:frames], est_poses)
            t_err, r_err = val.rpe()
            objective_val = t_err + rotation_weight * r_err

            trial.set_user_attr("translation_err_pct",   round(t_err, 4))
            trial.set_user_attr("rotation_err_deg_100m", round(r_err, 4))
            trial.set_user_attr("objective",             round(objective_val, 4))

            print(
                f"  Trial {trial.number:3d} | "
                f"t_err={t_err:.3f}%  r_err={r_err:.3f} deg/100m  "
                f"obj={objective_val:.4f}  ({elapsed:.1f}s)"
            )
            return objective_val

        finally:
            for f in [tmp_cfg, tmp_poses]:
                if os.path.exists(f):
                    os.remove(f)

    return objective


def save_best_config(study: optuna.Study, base_cfg: dict) -> str:
    best = study.best_params
    best_cfg = base_cfg.copy()
    best_cfg.update({
        "VOXEL_LEAF_SIZE":             best["voxel_leaf_size"],
        "MAX_CORRESPONDENCE_DISTANCE": best["max_correspondence_distance"],
        "MAX_ITERATIONS":              best["max_iterations"],
        "NORMAL_K_SEARCH":             best["normal_k_search"],
        "TRANSFORMATION_EPSILON":      best["transformation_epsilon"],
        "EUCLIDEAN_FITNESS_EPSILON":   best["euclidean_fitness_epsilon"],
        
        "EKF_PROCESS_NOISE":           best["ekf_process_noise"],
        "EKF_GPS_NOISE":               best["ekf_gps_noise"],
        "EKF_ICP_NOISE":               best["ekf_icp_noise"],
    })
    best_cfg["TOTAL_FRAMES_TO_PROCESS"] = base_cfg.get("TOTAL_FRAMES_TO_PROCESS", 1000)

    out_path = os.path.join(PROJECT_ROOT, "config", "config_best.json")
    with open(out_path, "w") as f:
        json.dump(best_cfg, f, indent=4)
    print(f"\n[OK] Best config saved -> {out_path}")
    return out_path


def save_study_csv(study: optuna.Study):
    df = study.trials_dataframe()
    out_path = os.path.join(PROJECT_ROOT, "optuna_results.csv")
    df.to_csv(out_path, index=False)
    print(f"[OK] Study results saved -> {out_path}")


def save_plots(study: optuna.Study):
    out_dir = os.path.join(PROJECT_ROOT, "optuna_plots")
    os.makedirs(out_dir, exist_ok=True)
    try:
        for fn, name in [
            (plot_optimization_history, "optimization_history"),
            (plot_param_importances,    "param_importances"),
            (plot_parallel_coordinate,  "parallel_coordinate"),
        ]:
            fig = fn(study)
            fig.write_html(os.path.join(out_dir, f"{name}.html"))
        print(f"[OK] Plots saved -> {out_dir}/")
    except Exception as e:
        print(f"[WARN] Could not save plots: {e}")


def parse_args():
    p = argparse.ArgumentParser(description="Optuna ICP hyperparameter tuner")
    p.add_argument("--n-trials",        type=int,   default=50)
    p.add_argument("--frames",          type=int,   default=200,
                   help="Frames per trial (default: 200)")
    p.add_argument("--binary",          type=str,
                   default=os.path.join(PROJECT_ROOT, "build", "bin", "lidar_localization"),
                   help="Path to compiled C++ binary")
    p.add_argument("--rotation-weight", type=float, default=1.0,
                   help="Weight lambda for rotation error in objective (default: 1.0)")
    p.add_argument("--study-name",      type=str,   default="icp_tuning")
    p.add_argument("--storage",         type=str,   default=None,
                   help="Optuna storage URL e.g. sqlite:///optuna.db")
    p.add_argument("--n-jobs",          type=int,   default=1)
    return p.parse_args()


def main():
    args     = parse_args()
    base_cfg = load_base_config()

    print("Loading GT poses...")
    gt_poses = KittiDataLoader.load_gt_poses(base_cfg["GT_POSE_PATH"])
    print(f"  Loaded {len(gt_poses)} GT frames.")

    print(f"\nStarting Optuna study '{args.study_name}'")
    print(f"  n_trials={args.n_trials}  frames_per_trial={args.frames}  rotation_weight={args.rotation_weight}")
    print(f"  Binary : {args.binary}\n")

    sampler = optuna.samplers.TPESampler(seed=42)
    study   = optuna.create_study(
        study_name   = args.study_name,
        direction    = "minimize",
        sampler      = sampler,
        storage      = args.storage,
        load_if_exists = True,
    )

    objective = make_objective(
        base_cfg        = base_cfg,
        binary_path     = args.binary,
        gt_poses        = gt_poses,
        frames          = args.frames,
        rotation_weight = args.rotation_weight,
    )

    study.optimize(objective, n_trials=args.n_trials, n_jobs=args.n_jobs)

    print("\n" + "="*50)
    print(" BEST TRIAL")
    print("="*50)
    best = study.best_trial
    print(f"  Number    : {best.number}")
    print(f"  Objective : {best.value:.4f}")
    print(f"  t_err     : {best.user_attrs.get('translation_err_pct', 'N/A')} %")
    print(f"  r_err     : {best.user_attrs.get('rotation_err_deg_100m', 'N/A')} deg/100m")
    print("\n  Parameters:")
    for k, v in best.params.items():
        print(f"    {k:35s}: {v}")
    print("="*50)

    save_best_config(study, base_cfg)
    save_study_csv(study)
    save_plots(study)


if __name__ == "__main__":
    main()
