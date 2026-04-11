import numpy as np
import argparse
import json
import os
from kitti import KittiDataLoader
from visualizer import Visualizer
from visualizer import PCDRenderer
PROJECT_ROOT = "/mnt/d/DeepLearning/Lidar-Localization"

class Validator:
    def __init__(self, gt_poses, est_poses):
        self.length = min(len(gt_poses), len(est_poses))
        self.gt = gt_poses[:self.length]
        self.est = est_poses[:self.length]

    def evaluate_metrics(self):
        """Calculates Frame-to-Frame Relative Pose Error (RPE)"""
        t_errs, r_errs = [], []
        total_dist = 0.0

        for i in range(1, self.length):
            # Calculate physical distance driven in GT
            dist = np.linalg.norm(self.gt[i][:3, 3] - self.gt[i-1][:3, 3])
            total_dist += dist

            if dist < 0.1:  # Skip tiny movements to avoid division by zero
                continue

            # Delta GT
            T_gt_i_inv = np.linalg.inv(self.gt[i-1])
            delta_gt = T_gt_i_inv @ self.gt[i]

            # Delta Est
            T_est_i_inv = np.linalg.inv(self.est[i-1])
            delta_est = T_est_i_inv @ self.est[i]

            # Error Matrix: Difference between the two deltas
            error_matrix = np.linalg.inv(delta_gt) @ delta_est

            # Extract Translation Error
            t_err = np.linalg.norm(error_matrix[:3, 3])
            t_errs.append(t_err / dist)

            # Extract Rotation Error
            trace = np.trace(error_matrix[:3, :3])
            val = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
            r_err = np.arccos(val)
            r_errs.append(r_err / dist)

        # Averages
        avg_t_err_pct = np.mean(t_errs) * 100 if t_errs else 0.0
        avg_r_err_deg_100m = np.degrees(np.mean(r_errs)) * 100 if r_errs else 0.0

        print("\n" + "="*40)
        print(" KITTI VALIDATION METRICS (RPE)")
        print("="*40)
        print(f" Frames Evaluated : {self.length}")
        print(f" Distance Driven  : {total_dist:.2f} meters")
        print(f" Translation Error: {avg_t_err_pct:.3f} %")
        print(f" Rotation Error   : {avg_r_err_deg_100m:.3f} deg/100m")
        print("="*40 + "\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize and Validate KITTI Trajectory")
    parser.add_argument("--plot-pcd", action="store_true", default=True)
    parser.add_argument("--no-pcd", action="store_false", dest="plot_pcd")
    parser.add_argument("--plot-gt", action="store_true", default=True)
    parser.add_argument("--no-gt", action="store_false", dest="plot_gt")
    parser.add_argument("--plot-pred", action="store_true", default=True)
    parser.add_argument("--no-pred", action="store_false", dest="plot_pred")
    parser.add_argument("-o", "--output", default="trajectory_side_by_side.mp4")
    args = parser.parse_args()
    
    # Load Configuration
    config_path = f"{PROJECT_ROOT}/config/config.json"
    with open(config_path, 'r') as f:
        config = json.load(f)

    gt_pose_path = config['GT_POSE_PATH']
    pcd_folder = config['PCD_INPUT_PATH']
    est_pose_path = config.get('POSES_OUTPUT_PATH', 'estimated_poses.txt')
    
    if not os.path.isabs(est_pose_path):
        est_pose_path = os.path.join(f"{PROJECT_ROOT}/build/bin", est_pose_path)

    gt_poses = None
    if args.plot_gt:
        print(f"Loading GT poses...")
        gt_poses = KittiDataLoader.load_gt_poses(gt_pose_path)

    est_poses = None
    if args.plot_pred:
        if os.path.exists(est_pose_path):
            print(f"Loading Estimated poses...")
            est_poses = KittiDataLoader.load_estimated_poses(est_pose_path)
        else:
            print(f"Warning: Estimated poses not found at {est_pose_path}")

    if gt_poses and est_poses:
        validator = Validator(gt_poses, est_poses)
        validator.evaluate_metrics()

    try:
        pcd_renderer = PCDRenderer(pcd_folder, enabled=args.plot_pcd)
        vis = Visualizer(gt_poses, est_poses, pcd_renderer)
        vis.render(output_filename=args.output)
    except Exception as e:
        print(f"Critical Error: {e}")