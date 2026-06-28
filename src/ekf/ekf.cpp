#include "ekf.h"
#include <iostream>

EKF::EKF() : initialized_(false) {
    x_ = Eigen::VectorXd::Zero(9);
    P_ = Eigen::MatrixXd::Identity(9, 9) * 1.0;
}

void EKF::init(const Eigen::VectorXd& x0) {
    x_ = x0;
    P_ = Eigen::MatrixXd::Identity(9, 9) * 0.1;
    initialized_ = true;
}

Eigen::VectorXd EKF::motionModel(const Eigen::VectorXd& x, double dt, double vf, double vl, double vu, double wf, double wl, double wu) const {
    Eigen::VectorXd x_new = x;

    double roll = x(6);
    double pitch = x(7);
    double yaw = x(8);

    // Rotation matrix from body to world frame
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f R = (yawAngle * pitchAngle * rollAngle).matrix();

    Eigen::Vector3f v_body(vf, vl, vu);
    Eigen::Vector3f v_world = R * v_body;

    // Update position
    x_new(0) += v_world(0) * dt;
    x_new(1) += v_world(1) * dt;
    x_new(2) += v_world(2) * dt;

    // Update velocity in state (for completeness)
    x_new(3) = v_world(0);
    x_new(4) = v_world(1);
    x_new(5) = v_world(2);

    // Update orientation (simple Euler integration)
    x_new(6) += wf * dt;
    x_new(7) += wl * dt;
    x_new(8) += wu * dt;

    // Normalize angles
    while (x_new(6) > M_PI) x_new(6) -= 2.0 * M_PI;
    while (x_new(6) < -M_PI) x_new(6) += 2.0 * M_PI;
    while (x_new(7) > M_PI) x_new(7) -= 2.0 * M_PI;
    while (x_new(7) < -M_PI) x_new(7) += 2.0 * M_PI;
    while (x_new(8) > M_PI) x_new(8) -= 2.0 * M_PI;
    while (x_new(8) < -M_PI) x_new(8) += 2.0 * M_PI;

    return x_new;
}

void EKF::predictIMU(double dt, double process_noise, double vf, double vl, double vu, double wf, double wl, double wu) {
    if (!initialized_) return;

    // Numerical Jacobian computation (Finite Differences)
    int n = x_.size();
    Eigen::MatrixXd F_j = Eigen::MatrixXd::Zero(n, n);
    double eps = 1e-5;

    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd x_plus = x_;
        x_plus(i) += eps;
        Eigen::VectorXd x_minus = x_;
        x_minus(i) -= eps;

        Eigen::VectorXd f_plus = motionModel(x_plus, dt, vf, vl, vu, wf, wl, wu);
        Eigen::VectorXd f_minus = motionModel(x_minus, dt, vf, vl, vu, wf, wl, wu);

        // Central difference
        // Need to be careful with angle differences
        Eigen::VectorXd diff = f_plus - f_minus;
        for (int k = 6; k < 9; ++k) {
            while (diff(k) > M_PI) diff(k) -= 2.0 * M_PI;
            while (diff(k) < -M_PI) diff(k) += 2.0 * M_PI;
        }

        F_j.col(i) = diff / (2.0 * eps);
    }

    // Process noise covariance Q
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(9, 9) * process_noise;

    // Predict state
    x_ = motionModel(x_, dt, vf, vl, vu, wf, wl, wu);

    // Predict covariance
    P_ = F_j * P_ * F_j.transpose() + Q;
}

void EKF::updateICP(const Eigen::VectorXd& z_icp, double icp_noise) {
    if (!initialized_) return;

    // Measurement matrix H for ICP [px, py, pz, roll, pitch, yaw]
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 9);
    H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1; // Position
    H(3, 6) = 1; H(4, 7) = 1; H(5, 8) = 1; // Orientation

    // Measurement noise covariance R
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6, 6) * icp_noise;

    // Innovation
    Eigen::VectorXd y = z_icp - H * x_;

    // Innovation covariance
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;

    // Kalman gain
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // Update state
    x_ = x_ + K * y;

    // Update covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);
    P_ = (I - K * H) * P_;
}

void EKF::updateGPS(const Eigen::Vector3d& z_gps, double gps_noise) {
    if (!initialized_) return;

    // Measurement matrix H for GPS [px, py, pz]
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 9);
    H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1; // Position only

    // Measurement noise covariance R
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * gps_noise;

    // Innovation
    Eigen::VectorXd y = z_gps - H * x_;

    // Innovation covariance
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;

    // Kalman gain
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // Update state
    x_ = x_ + K * y;

    // Update covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);
    P_ = (I - K * H) * P_;
}


