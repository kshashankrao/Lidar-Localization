import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import sys
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "config.json")

sys.path.insert(0, SCRIPT_DIR)
from kitti import KittiDataLoader

def main():
    print("Loading GT poses...")
    with open(CONFIG_PATH, "r") as f:
        config = json.load(f)
        
    gt_poses_all = KittiDataLoader.load_gt_poses(config["GT_POSE_PATH"])
    
    methods = ["ICP", "EKF_GPS", "EKF_ICP"]
    est_poses_dict = {}
    
    for m in methods:
        p = os.path.join(PROJECT_ROOT, f"poses_{m}.txt")
        if os.path.exists(p):
            est_poses_dict[m] = KittiDataLoader.load_estimated_poses(p)
        else:
            print(f"File not found: {p}")
            return
            
    num_frames = len(est_poses_dict["ICP"])
    gt_poses = gt_poses_all[:num_frames]
    
    gt_xy = np.array([[p[0,3], p[1,3]] for p in gt_poses])
    icp_xy = np.array([[p[0,3], p[1,3]] for p in est_poses_dict["ICP"]])
    gps_xy = np.array([[p[0,3], p[1,3]] for p in est_poses_dict["EKF_GPS"]])
    imu_xy = np.array([[p[0,3], p[1,3]] for p in est_poses_dict["EKF_ICP"]])
    
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title("Lidar Localization Comparison (ICP vs EKF_GPS vs EKF_ICP)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True)
    
    min_x, max_x = np.min(gt_xy[:,0]) - 50, np.max(gt_xy[:,0]) + 50
    min_y, max_y = np.min(gt_xy[:,1]) - 50, np.max(gt_xy[:,1]) + 50
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    
    line_gt, = ax.plot([], [], 'k-', label='Ground Truth', linewidth=2)
    line_icp, = ax.plot([], [], 'r--', label='ICP Only', alpha=0.8)
    line_gps, = ax.plot([], [], 'g-', label='EKF IMU + GPS', alpha=0.8)
    line_imu, = ax.plot([], [], 'b-', label='EKF IMU + ICP', alpha=0.8)
    
    point_gt, = ax.plot([], [], 'ko')
    point_icp, = ax.plot([], [], 'ro')
    point_gps, = ax.plot([], [], 'go')
    point_imu, = ax.plot([], [], 'bo')
    
    ax.legend()
    
    def init():
        line_gt.set_data([], [])
        line_icp.set_data([], [])
        line_gps.set_data([], [])
        line_imu.set_data([], [])
        point_gt.set_data([], [])
        point_icp.set_data([], [])
        point_gps.set_data([], [])
        point_imu.set_data([], [])
        return line_gt, line_icp, line_gps, line_imu, point_gt, point_icp, point_gps, point_imu

    def update(frame):
        f = min(frame * 5, num_frames - 1)
        
        line_gt.set_data(gt_xy[:f, 0], gt_xy[:f, 1])
        line_icp.set_data(icp_xy[:f, 0], icp_xy[:f, 1])
        line_gps.set_data(gps_xy[:f, 0], gps_xy[:f, 1])
        line_imu.set_data(imu_xy[:f, 0], imu_xy[:f, 1])
        
        point_gt.set_data([gt_xy[f, 0]], [gt_xy[f, 1]])
        point_icp.set_data([icp_xy[f, 0]], [icp_xy[f, 1]])
        point_gps.set_data([gps_xy[f, 0]], [gps_xy[f, 1]])
        point_imu.set_data([imu_xy[f, 0]], [imu_xy[f, 1]])
        
        return line_gt, line_icp, line_gps, line_imu, point_gt, point_icp, point_gps, point_imu
        
    frames_to_animate = num_frames // 5
    print(f"Animating {frames_to_animate} frames...")
    
    ani = FuncAnimation(fig, update, frames=frames_to_animate, init_func=init, blit=True)
    
    out_dir = os.path.join(PROJECT_ROOT, "readme")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "comparison.mp4")
    
    print("Saving mp4...")
    ani.save(out_path, fps=30, extra_args=['-vcodec', 'libx264'])
    print(f"Saved to {out_path}")

if __name__ == "__main__":
    main()
