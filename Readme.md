# LIDAR Localization

## Description
This project implements a LIDAR based localization using the KITTI datasets. 

The algorithm includes:

1. ICP point-to-plane pose estimation. [IMPLEMENTED]
2. Extended Kalman filter (EKF) to fuse LiDAR, GPS, and IMU pose estimation. [IMPLEMENTED]
3. Optuna-based automated hyperparameter optimization. [IMPLEMENTED]

## Algorithm Deep-Dives

| Algorithm | Readme |
|-----------|--------|
| **ICP** — Point-to-plane iterative closest point, data association, SVD pose estimation | [readme/icp.md](readme/icp.md) |
| **EKF** — Predict/update cycle, covariance, innovation, Jacobians, sensor fusion, parameter tuning | [readme/ekf.md](readme/ekf.md) |

## Usage

1. Clone the repository and navigate to the project directory.
2. Install the required dependencies (PCL, OpenCV, nlohmann/json). 
   ```bash
   ./setup.sh
   ``` 
3. Set the data path in `config/config.json`. You can also set `LOCALIZATION_METHOD` to `"ICP"`, `"EKF_GPS"`, or `"EKF_ICP"`.
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

Frames Evaluated: 1000

| Configuration | Translation Error (%) | Rotation Error (deg/100m) |
| :--- | :--- | :--- |
| **ICP Only** | 6.686 | 16.258 |
| **EKF IMU + GPS** | 141.793 | 124.034 |
| **EKF IMU + ICP** | 6.190 | 15.672 |

## Demo

![Trajectory Comparison](readme/comparison.gif)

### Observation: Parameter Tuning vs Evaluation

* **Parameter Optimization:** Using Optuna, the pipeline was specifically optimized for the **EKF IMU + ICP** configuration. As a result, the IMU-driven EKF achieved the best translation (6.19%) and rotation (15.67 deg/100m) errors!
* **Degradation of un-tuned methods:** Because the noise parameters (e.g. `EKF_GPS_NOISE`) were jointly optimized for the IMU-ICP architecture, running the `EKF_GPS` method using these exact same noise parameters resulted in severe degradation. This highlights the importance of tuning filter parameters specifically for the active sensor configuration!

## Future Work
1. Run an independent Optuna parameter sweep specifically to tune the `EKF_GPS` method.
2. Evaluate the pipeline on the full KITTI Odometry Benchmark dataset.
