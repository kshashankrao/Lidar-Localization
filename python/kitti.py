import os
import glob
import numpy as np

class KittiDataLoader:
    @staticmethod
    def load_estimated_poses(pose_file):
        """Load 3x4 matrices saved by the C++ ICP class"""
        poses = []
        with open(pose_file, 'r') as f:
            for line in f:
                vals = np.fromstring(line, sep=' ')
                if len(vals) == 12:
                    pose = np.eye(4)
                    pose[:3, :] = vals.reshape(3, 4)
                    poses.append(pose)
        return poses

    @staticmethod
    def oxts_to_se3_poses(oxts_data):
        """Convert oxts data array to SE(3) poses, aligned to first frame's heading"""
        lat, lon, alt = oxts_data[:, 0], oxts_data[:, 1], oxts_data[:, 2]
        roll, pitch, yaw = oxts_data[:, 3], oxts_data[:, 4], oxts_data[:, 5]
        
        R_earth = 6371000  
        lat_0, lon_0 = lat[0], lon[0]
        
        x_gps = R_earth * np.cos(np.radians(lat_0)) * np.radians(lon - lon_0)
        y_gps = R_earth * np.radians(lat - lat_0)
        z = alt - alt[0]
        
        initial_yaw = yaw[0]
        cos_0, sin_0 = np.cos(-initial_yaw), np.sin(-initial_yaw)
        R_align = np.array([[cos_0, -sin_0], [sin_0, cos_0]])

        poses = []
        for i in range(len(oxts_data)):
            local_xy = R_align @ np.array([x_gps[i], y_gps[i]])
            
            cy, sy = np.cos(yaw[i] - initial_yaw), np.sin(yaw[i] - initial_yaw)
            cp, sp = np.cos(pitch[i]), np.sin(pitch[i])
            cr, sr = np.cos(roll[i]), np.sin(roll[i])
            
            R = np.array([
                [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                [-sp, cp*sr, cp*cr]
            ])
            
            pose = np.eye(4)
            pose[:3, :3] = R
            pose[0, 3] = local_xy[0]
            pose[1, 3] = local_xy[1]
            pose[2, 3] = z[i]
            poses.append(pose)
        
        return poses

    @staticmethod
    def load_gt_poses(pose_dir):
        """Load KITTI oxts format poses"""
        pose_files = sorted(glob.glob(os.path.join(pose_dir, "*.txt")))
        if not pose_files:
            raise ValueError(f"No pose files found in {pose_dir}")
        
        oxts_data = []
        for pose_file in pose_files:
            with open(pose_file, 'r') as f:
                line = f.readline().strip()
                values = np.array([float(x) for x in line.split()])
                oxts_data.append(values)
                
        return KittiDataLoader.oxts_to_se3_poses(np.array(oxts_data))