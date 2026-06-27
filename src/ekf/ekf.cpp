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

void EKF::predict(double dt, double process_noise) {
    if (!initialized_) return;

    // State transition matrix F
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
    F(0, 3) = dt; // px = px + vx*dt
    F(1, 4) = dt; // py = py + vy*dt
    F(2, 5) = dt; // pz = pz + vz*dt

    // Process noise covariance Q
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(9, 9) * process_noise;

    // Predict state
    x_ = F * x_;

    // Predict covariance
    P_ = F * P_ * F.transpose() + Q;
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

void EKF::updateIMU(const Eigen::Vector3d& z_imu, double imu_noise) {
    if (!initialized_) return;

    // Measurement matrix H for IMU [roll, pitch, yaw]
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 9);
    H(0, 6) = 1; H(1, 7) = 1; H(2, 8) = 1; // Orientation only

    // Measurement noise covariance R
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * imu_noise;

    // Innovation
    Eigen::VectorXd y = z_imu - H * x_;

    // Normalize yaw difference to [-pi, pi]
    while (y(2) > M_PI) y(2) -= 2.0 * M_PI;
    while (y(2) < -M_PI) y(2) += 2.0 * M_PI;

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
