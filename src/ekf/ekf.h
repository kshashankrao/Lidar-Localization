#pragma once

#include <Eigen/Dense>

class EKF {
public:
    EKF();

    void init(const Eigen::VectorXd& x0);

    // Predict step using IMU body velocities and angular rates
    void predictIMU(double dt, double process_noise, double vf, double vl, double vu, double wf, double wl, double wu);

    // Update with ICP measurement [x, y, z, roll, pitch, yaw]
    void updateICP(const Eigen::VectorXd& z_icp, double icp_noise);

    // Update with GPS measurement [x, y, z]
    void updateGPS(const Eigen::Vector3d& z_gps, double gps_noise);

    Eigen::VectorXd getState() const { return x_; }
    void setState(const Eigen::VectorXd& x) { x_ = x; }

private:
    Eigen::VectorXd motionModel(const Eigen::VectorXd& x, double dt, double vf, double vl, double vu, double wf, double wl, double wu) const;

    Eigen::VectorXd x_; // State: [px, py, pz, vx, vy, vz, roll, pitch, yaw]^T (9x1)
    Eigen::MatrixXd P_; // Covariance (9x9)

    bool initialized_;
};
