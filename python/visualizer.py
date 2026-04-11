import cv2
import numpy as np
import glob
import open3d as o3d

class Visualizer:
    def __init__(self, gt_poses=None, est_poses=None, pcd_renderer=None, width=800, height=800):
        self.gt_poses = gt_poses
        self.est_poses = est_poses
        self.pcd_renderer = pcd_renderer
        self.width = width
        self.height = height

        self.gt_traj = self._extract_trajectory(gt_poses)
        self.est_traj = self._extract_trajectory(est_poses)
        self.gt_yaws = self._extract_yaws(gt_poses)
        self.est_yaws = self._extract_yaws(est_poses)

    def _extract_trajectory(self, poses):
        if poses is None: return None
        return np.array([[p[0, 3], p[1, 3]] for p in poses])

    def _extract_yaws(self, poses):
        if poses is None: return None
        return [np.degrees(np.arctan2(p[1, 0], p[0, 0])) for p in poses]

    def _draw_panel(self, poses, trajectory, traj_center, scale, title, color, frame_idx):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        off_x, off_y = self.width // 2, (self.height - 250) // 2 

        # Draw trajectory
        for j in range(1, frame_idx + 1):
            x1 = int((trajectory[j-1, 0] - traj_center[0]) * scale) + off_x
            y1 = int((trajectory[j-1, 1] - traj_center[1]) * scale) + off_y
            x2 = int((trajectory[j, 0] - traj_center[0]) * scale) + off_x
            y2 = int((trajectory[j, 1] - traj_center[1]) * scale) + off_y
            cv2.line(img, (x1, y1), (x2, y2), color, 2)
            
        # Draw car position
        if frame_idx < len(trajectory):
            cx = int((trajectory[frame_idx, 0] - traj_center[0]) * scale) + off_x
            cy = int((trajectory[frame_idx, 1] - traj_center[1]) * scale) + off_y
            cv2.circle(img, (cx, cy), 6, (255, 255, 255), -1)
            
        cv2.putText(img, title, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        # Draw Matrix
        if frame_idx < len(poses):
            pose = poses[frame_idx]
            start_y = self.height - 180
            cv2.putText(img, "Current Pose Matrix:", (20, start_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            for r in range(4):
                row_str = f"[{pose[r,0]:8.3f}, {pose[r,1]:8.3f}, {pose[r,2]:8.3f}, {pose[r,3]:8.3f}]"
                cv2.putText(img, row_str, (20, start_y + 35 + (r * 30)), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.2, (255, 255, 255), 1)
                
        cv2.rectangle(img, (0, 0), (self.width-1, self.height-1), (50, 50, 50), 2)
        return img

    def render(self, output_filename="trajectory_side_by_side.mp4", fps=10):
        print("Rendering video")
        lengths = [len(p) for p in [self.est_poses, self.gt_poses, self.pcd_renderer.pcd_files] if p is not None and len(p) > 0]
        if not lengths:
            raise ValueError("No data available to render")
            
        num_frames = min(lengths)
        
        # Setup Video dimensions based on active features
        panels_count = sum([self.pcd_renderer.enabled, self.gt_poses is not None, self.est_poses is not None])
        total_width = self.width * panels_count
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video = cv2.VideoWriter(output_filename, fourcc, fps, (total_width, self.height))

        # Center calculation
        ref_traj = self.gt_traj if self.gt_traj is not None else self.est_traj
        traj_center = np.mean(ref_traj[:num_frames], axis=0) if ref_traj is not None else np.array([0, 0])
        scale = 0.5 

        for i in range(num_frames):
            panels = []
            if self.pcd_renderer.enabled:
                panels.append(self.pcd_renderer.get_frame(i))
            if self.gt_poses is not None:
                panels.append(self._draw_panel(self.gt_poses, self.gt_traj, traj_center, scale, "Ground Truth", (255, 100, 100), i))
            if self.est_poses is not None:
                panels.append(self._draw_panel(self.est_poses, self.est_traj, traj_center, scale, "Estimated (ICP)", (100, 100, 255), i))

            combined = np.hstack(panels)
            cv2.putText(combined, f"Frame: {i} / {num_frames-1}", (total_width // 2 - 100, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            video.write(combined)
            # if i % 20 == 0:
            #     print(f"Frame {i}/{num_frames} encoded.")

        video.release()
        print(f"Successfully saved video to: {output_filename}")

class PCDRenderer:
    def __init__(self, pcd_folder, width=800, height=800, enabled=True):
        self.enabled = enabled
        self.width = width
        self.height = height
        
        if self.enabled:
            self.pcd_files = sorted(glob.glob(os.path.join(pcd_folder, "*.pcd")))
            self.render = o3d.visualization.rendering.OffscreenRenderer(self.width, self.height)
            self.mtl = o3d.visualization.rendering.MaterialRecord()
            self.mtl.base_color = [1.0, 1.0, 1.0, 1.0]
            self.mtl.shader = "defaultUnlit"
            self.mtl.point_size = 2.0
            
            center, eye, up = [0, 0, 0], [0, 0, 80], [0, 1, 0]
            self.render.setup_camera(60.0, center, eye, up)
            self.render.scene.set_background([0.05, 0.05, 0.05, 1.0])
        else:
            self.pcd_files = []
            self.render = None

    def get_frame(self, frame_idx):
        if not self.enabled or frame_idx >= len(self.pcd_files):
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
            
        pcd = o3d.io.read_point_cloud(self.pcd_files[frame_idx])
        self.render.scene.remove_geometry("current_scan")
        self.render.scene.add_geometry("current_scan", pcd, self.mtl)
        
        img_o3d = self.render.render_to_image()
        cloud_img = cv2.cvtColor(np.asarray(img_o3d), cv2.COLOR_RGB2BGR)
        cv2.putText(cloud_img, "LiDAR View", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return cloud_img
