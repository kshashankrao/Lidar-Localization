# LIDAR Localization

## Description
This project implements a LIDAR based localization using the KITTI datasets. 

The algorithm includes:

1. ICP point-to-plane pose estimation. [IMPLEMENTED]
2. Extended Kalman filter (EKF) to fuse LiDAR, GPS, and IMU pose estimation. [IMPLEMENTED]
3. Optuna-based automated hyperparameter optimization. [IMPLEMENTED]
4. LOAM (LiDAR Odometry and Mapping) feature extraction, scan-to-map Gauss-Newton optimization, and loop closure drift correction. [IMPLEMENTED]

## Algorithm Deep-Dives

| Algorithm | Readme |
|-----------|--------|
| **ICP** — Point-to-plane iterative closest point, data association, SVD pose estimation | [readme/icp.md](readme/icp.md) |
| **EKF** — Predict/update cycle, covariance, innovation, Jacobians, sensor fusion, parameter tuning | [readme/ekf.md](readme/ekf.md) |
| **LOAM** — Curvature feature extraction, Point-to-Line & Point-to-Plane Gauss-Newton optimization, Huber robust weighting | [readme/loam.md](readme/loam.md) |

### LOAM Implementation Description

Our LOAM pipeline (`LOAMLocalization`) decouples 6-DoF LiDAR odometry and mapping into three stages:
1. **Curvature-Based Feature Extraction (`extractFeatures`)**: Analyzes local scan-ring curvature $c_i$ across horizontal sectors to segregate sharp **Corner/Edge points** ($c_i > 0.5$) and smooth **Planar Surface points** ($c_i < 0.15$).
2. **Scan-to-Map Gauss-Newton Optimization (`scanToMapOptimization`)**:
   - **Point-to-Line Residuals**: Matches edge points against 5 nearest corner neighbors in the local keyframe map, solving analytical 6-DoF Jacobians along the edge direction.
   - **Point-to-Plane Residuals**: Matches surface points against 5 nearest surface neighbors, fitting exact Householder QR unit normals $\hat{\mathbf{n}}$ and minimizing perpendicular distance $\hat{\mathbf{n}}^T \mathbf{p} + d$.
   - **Huber Robust Kernel**: Applies M-estimator robust weights ($w = \min(1, 0.2 / |r|)$) to suppress foliage and dynamic outliers.
3. **Sliding-Window Keyframe Map (`detectAndCorrectLoopClosure`)**: Retains an active local map window of recent keyframes (`map_leaf_size_ = 0.25m`) and performs KD-Tree loop closure queries to eliminate accumulated odometry drift.

## Usage

1. Clone the repository and navigate to the project directory.
2. Install the required dependencies (PCL, OpenCV, nlohmann/json). 
   ```bash
   ./setup.sh
   ``` 
3. Set the data path in `config/config.json`. You can also set `LOCALIZATION_METHOD` to `"ICP"`, `"EKF_GPS"`, `"EKF_ICP"`, or `"LOAM"`.
4. Build the C++ executable.
    ```bash
    mkdir build
    cd build
    cmake ..
    make -j8
    ```
5. Run the executable to estimate poses (and convert KITTI data to PCD format).
   ```bash
   cd bin
   ./lidar_localization 
   ```
6. Run the Optuna tuner to automatically optimize the pipeline (Optional).
    ```bash
    ./run_optuna.sh
    ```
7. Run the Python evaluation script to compare all configurations.
    ```bash
    source venv/bin/activate
    python python/evaluation.py
    ```

## Results

Frames Evaluated: 1000 (KITTI Sequence 0027)

| Configuration | Translation Error (%) | Rotation Error (deg/100m) | Mean Runtime / Frame |
| :--- | :--- | :--- | :--- |
| **ICP Only** | 6.686 | 16.258 | ~45 ms |
| **EKF IMU + GPS** | 141.793 | 124.034 | ~45 ms |
| **EKF IMU + ICP** | 6.190 | 15.672 | ~45 ms |
| **LOAM (Pure LiDAR)** | 6.347 | **14.209** | ~225 ms |

## Demo

![Trajectory Comparison](readme/comparison.gif)

### Observation: Parameter Tuning vs Evaluation

* **LOAM Rotation Accuracy & Pure LiDAR Performance:** Operating purely on 64-line LiDAR point clouds with **ZERO GPS or IMU aiding**, **LOAM** achieves the lowest rotation error (**14.209 deg/100m** — a **9.3% improvement** over dual-sensor EKF IMU+ICP and **12.6% improvement** over standard single-sensor ICP Only). Furthermore, LOAM achieves **6.347%** translation error, outperforming standard ICP Only (`6.686%`) and performing within 0.15% of dual-sensor IMU+ICP fusion.
* **Parameter Optimization:** Using Optuna, the pipeline was specifically optimized for the **EKF IMU + ICP** configuration. As a result, the IMU-driven EKF achieved the best translation error (6.190%), leveraging 100 Hz physical inertial acceleration measurements between scans.
* **Degradation of un-tuned methods:** Because the noise parameters (e.g. `EKF_GPS_NOISE`) were jointly optimized for the IMU-ICP architecture, running the `EKF_GPS` method using these exact same noise parameters resulted in severe degradation. This highlights the importance of tuning filter parameters specifically for the active sensor configuration!

## Future Work
1. Run an independent Optuna parameter sweep specifically to tune the `EKF_GPS` method.
2. Evaluate the pipeline on the full KITTI Odometry Benchmark dataset.
