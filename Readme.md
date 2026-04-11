# LIDAR Localization

## Description
This project implements a LIDAR based localization using the KITTI datasets. 

The algorithm includes:

1. ICP point to plane pose estimation. [IMPLEMENTED]
2. Kalman filter to fuse the IMU and the LIDAR pose estimation. [TODO]

## Usage

1. Clone the repository and navigate to the project directory.
2. Install the required dependencies (PCL, OpenCV, nlohmann/json). 
   ```
   ./setup.sh
   ``` 
3. Set the data path in config.json
4. Build the C++ executable.
    ```
    mkdir build
    cd build
    cmake ..
    make
    ```
5. Run the executable to convert KITTI data to PCD format.
   ```
   cd bin
   ./lidar_localization 
   ```
6. Run the Python validator to evaluate the pose estimation and visualize the results.
    ```  
    python validator.py --no-pcd
    ```
## Results

Frames Evaluated : 1000

1. Only ICP
    - Distance Driven  : 711.37 meters
    - Translation Error: 7.077 %
    - Rotation Error   : 38.061 deg/100m

## Future Work
1. Implement Kalman filter to fuse IMU and LIDAR pose estimation.
2. Optimize ICP parameters for better accuracy.




