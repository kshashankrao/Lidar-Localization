# LOAM (LiDAR Odometry and Mapping) Deep-Dive

This document details our C++ implementation of **LOAM (LiDAR Odometry and Mapping)** (`LOAMLocalization`), designed for robust 6-DoF 3D LiDAR odometry and mapping without reliance on external GPS or IMU sensors.

---

## 1. Architectural Overview

Our LOAM pipeline decouples 3D LiDAR pose estimation into:
1. **Curvature-Based Feature Extraction**: Extracts sharp geometric features (corners/edges) and planar surface patches from raw 64-line point clouds.
2. **Scan-to-Map Gauss-Newton Optimization**: Jointly optimizes 6-DoF pose ($t_x, t_y, t_z, roll, pitch, yaw$) against a voxel-downsampled local keyframe map using Point-to-Line and Point-to-Plane geometric constraints.
3. **Sliding-Window Keyframe Mapping & Loop Closure**: Maintains an active local map and periodically detects loop closures via KD-Tree spatial queries to eliminate accumulated odometry drift.

```
Raw Point Cloud (KITTI 64-line)
       │
       ▼
┌────────────────────────────────────────────────────────┐
│ 1. Feature Extraction (extractFeatures)                │
│    ├── Compute Local Curvature c_i                     │
│    ├── Sort points within horizontal scan sectors      │
│    ├── Extract Corner Cloud (c_i > c_thresh_high)      │
│    └── Extract Surface Cloud (c_i < c_thresh_low)      │
└────────────────────────────────────────────────────────┘
       │
       ▼
┌────────────────────────────────────────────────────────┐
│ 2. Initial Pose Prediction (processFrame)              │
│    └── Constant Velocity Model: T_pred = T_prev * ΔT   │
└────────────────────────────────────────────────────────┘
       │
       ▼
┌────────────────────────────────────────────────────────┐
│ 3. Scan-to-Map Gauss-Newton Optimization               │
│    ├── Point-to-Line Residuals (Corner KD-Tree search) │
│    ├── Point-to-Plane Residuals (Surface QR normal)    │
│    ├── Huber Robust Loss Outlier Rejection             │
│    └── Solve Normal Equations: (J^T W J) Δx = -J^T W r │
└────────────────────────────────────────────────────────┘
       │
       ▼
┌────────────────────────────────────────────────────────┐
│ 4. Keyframe Map Maintenance & Loop Closure             │
│    ├── Insert Keyframe if travel distance > 0.6m       │
│    └── ICP Loop Closure Drift Correction               │
└────────────────────────────────────────────────────────┘
```

---

## 2. Mathematical Formulation

### 2.1 Feature Extraction Curvature Metric
For each point $\mathbf{p}_i$ in the scan, local surface roughness/curvature $c_i$ is computed over a neighborhood $\mathcal{S}$ of adjacent laser points on the same scan ring:

$$c_i = \frac{1}{|\mathcal{S}| \cdot \|\mathbf{p}_i\|} \left\| \sum_{j \in \mathcal{S}, j \neq i} (\mathbf{p}_j - \mathbf{p}_i) \right\|$$

- Points with high curvature ($c_i > 0.5$) are classified as **Corner/Edge features**.
- Points with low curvature ($c_i < 0.15$) are classified as **Planar Surface features**.

---

### 2.2 Point-to-Line Residuals & Analytical Jacobians (Corner Features)
For a transformed corner point $\mathbf{p}_i^g$, we find its 5 nearest neighbors in the local corner map via KD-Tree. Covariance eigenvalue analysis confirms linear structure ($\lambda_3 > 3\lambda_2$). Given two points $\mathbf{p}_a, \mathbf{p}_b$ along the line segment, the point-to-line residual distance is:

$$r_{edge} = \frac{\| (\mathbf{p}_i^g - \mathbf{p}_a) \times (\mathbf{p}_i^g - \mathbf{p}_b) \|}{\| \mathbf{p}_a - \mathbf{p}_b \|}$$

The analytical 6-DoF Jacobian $\mathbf{J}_{edge} \in \mathbb{R}^{1 \times 6}$ with respect to translation $\mathbf{t}$ and rotation $\mathbf{R}$ is derived via:

$$\frac{\partial r_{edge}}{\partial \mathbf{t}} = \mathbf{u}_{line}^T, \quad \frac{\partial r_{edge}}{\partial \theta} = \left( (\mathbf{p}_i^g - \mathbf{t}) \times \mathbf{u}_{line} \right)^T$$

---

### 2.3 Point-to-Plane Residuals & Exact QR Unit Normals (Surface Features)
For a transformed surface point $\mathbf{p}_i^g$, we find its 5 nearest neighbors in the local surface map. We fit a local plane equation $\mathbf{n}^T \mathbf{x} + d = 0$ using Householder QR decomposition (`A.householderQr().solve(B_vec)`).

Normalizing $\mathbf{n}$ to a true unit vector $\hat{\mathbf{n}} = \mathbf{n} / \|\mathbf{n}\|$, the exact signed perpendicular distance from point $\mathbf{p}_i^g$ to the surface patch is:

$$r_{surf} = \hat{\mathbf{n}}^T \mathbf{p}_i^g + d$$

The analytical 6-DoF Jacobian $\mathbf{J}_{surf} \in \mathbb{R}^{1 \times 6}$ is:

$$\mathbf{J}_{surf} = \begin{bmatrix} \hat{\mathbf{n}}^T & \left( (\mathbf{p}_i^g - \mathbf{t}) \times \hat{\mathbf{n}} \right)^T \end{bmatrix}$$

---

### 2.4 Huber Robust Loss Weighting
To prevent outlier foliage, moving objects, or occlusion boundaries from distorting convergence, each residual is weighted by a Huber robust loss kernel:

$$w(r) = \begin{cases} 1.0 & \text{if } |r| < \delta \\ \frac{\delta}{|r|} & \text{if } |r| \ge \delta \end{cases} \quad (\delta = 0.2\text{m})$$

The Gauss-Newton normal equations are formed and solved iteratively:

$$\left( \sum w_k \mathbf{J}_k^T \mathbf{J}_k \right) \Delta \mathbf{x} = -\sum w_k \mathbf{J}_k^T r_k$$

---

## 3. Benchmark Performance (KITTI Sequence 0027)

| Configuration | Translation Error (%) | Rotation Error (deg/100m) | Mean Runtime / Frame |
| :--- | :--- | :--- | :--- |
| **ICP Only** | 6.686 | 16.258 | ~45 ms |
| **EKF IMU + ICP** | 6.190 | 15.672 | ~45 ms |
| **LOAM (Pure LiDAR)** | **6.347** | **14.209** | **~225 ms** |

- **Lowest Rotation Error**: Achieves **14.209 deg/100m**, outperforming both EKF IMU+ICP (15.672 deg/100m) and ICP Only (16.258 deg/100m) due to strict Corner/Edge orientation constraints.
- **Pure Single-Sensor Odometry**: Achieves **6.347%** translation error using only 64-line LiDAR point clouds with zero external GPS or IMU aiding.
