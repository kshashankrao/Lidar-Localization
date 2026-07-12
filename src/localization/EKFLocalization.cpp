#include "EKFLocalization.h"
#include <fstream>
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

EKFLocalization::EKFLocalization(float leaf_size,
                                 float max_correspondence_distance,
                                 int   max_iterations,
                                 int   normal_k_search,
                                 double transformation_epsilon,
                                 double euclidean_fitness_epsilon,
                                 double ekf_process_noise,
                                 double ekf_gps_noise,
                                 double ekf_icp_noise,
                                 const std::string& localization_method)
    : ekf_process_noise_(ekf_process_noise),
      ekf_gps_noise_(ekf_gps_noise),
      ekf_icp_noise_(ekf_icp_noise),
      localization_method_(localization_method),
      global_pose_(Eigen::Matrix4f::Identity()),
      first_gps_received_(false),
      random_generator_(std::random_device{}()),
      noise_dist_xy_(0.0, 3.0),
      noise_dist_z_(0.0, 10.0)
{
    icp_ = std::make_unique<PointToPlaneICP>(
        leaf_size, max_correspondence_distance, max_iterations,
        normal_k_search, transformation_epsilon, euclidean_fitness_epsilon
    );
}

Eigen::Vector3d EKFLocalization::convertGpsToLocal(double lat, double lon, double alt) {
    double R_earth = 6371000.0;
    
    double lat_rad = lat * M_PI / 180.0;
    double lat_0_rad = lat_0_ * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double lon_0_rad = lon_0_ * M_PI / 180.0;

    double x_gps = R_earth * std::cos(lat_0_rad) * (lon_rad - lon_0_rad);
    double y_gps = R_earth * (lat_rad - lat_0_rad);
    double z = alt - alt_0_;

    Eigen::Vector2d local_xy = R_align_ * Eigen::Vector2d(x_gps, y_gps);

    // Apply synthetic noise to simulate real consumer GPS
    // (We skip the very first frame to ensure the coordinate origin perfectly matches the GT origin)
    if (lat != lat_0_ || lon != lon_0_) {
        local_xy.x() += noise_dist_xy_(random_generator_);
        local_xy.y() += noise_dist_xy_(random_generator_);
        z += noise_dist_z_(random_generator_);
    }

    return Eigen::Vector3d(local_xy.x(), local_xy.y(), z);
}

void EKFLocalization::processFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
                                   const std::vector<double>& oxts_data)
{
    if (!first_gps_received_) {
        icp_->processFrame(current_scan, oxts_data);
        Eigen::Matrix4f icp_pose = icp_->getGlobalPose();

        if (oxts_data.size() >= 6) {
            lat_0_ = oxts_data[0];
            lon_0_ = oxts_data[1];
            alt_0_ = oxts_data[2];
            initial_yaw_ = oxts_data[5];
            double cos_0 = std::cos(-initial_yaw_);
            double sin_0 = std::sin(-initial_yaw_);
            R_align_ << cos_0, -sin_0,
                        sin_0,  cos_0;
        }

        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(9);
        x0(0) = icp_pose(0, 3);
        x0(1) = icp_pose(1, 3);
        x0(2) = icp_pose(2, 3);
        ekf_.init(x0);

        first_gps_received_ = true;
        global_pose_ = icp_pose;
        return;
    }

    // 1. IMU Prediction step: predict state using IMU linear velocities & angular rates
    double dt = 0.1;
    double vf = 0.0, vl = 0.0, vu = 0.0;
    double wf = 0.0, wl = 0.0, wu = 0.0;
    if (oxts_data.size() >= 23) {
        vf = oxts_data[8];
        vl = oxts_data[9];
        vu = oxts_data[10];
        wf = oxts_data[20];
        wl = oxts_data[21];
        wu = oxts_data[22];
    }
    ekf_.predictIMU(dt, ekf_process_noise_, vf, vl, vu, wf, wl, wu);

    // Convert IMU predicted state to prior pose for LiDAR corrector
    Eigen::VectorXd pred_state = ekf_.getState();
    Eigen::AngleAxisf rollAnglePred(pred_state(6), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAnglePred(pred_state(7), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAnglePred(pred_state(8), Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f T_pred = Eigen::Matrix4f::Identity();
    T_pred.block<3,3>(0,0) = (yawAnglePred * pitchAnglePred * rollAnglePred).matrix();
    T_pred(0,3) = static_cast<float>(pred_state(0));
    T_pred(1,3) = static_cast<float>(pred_state(1));
    T_pred(2,3) = static_cast<float>(pred_state(2));

    // Provide IMU prediction as prior to LiDAR corrector
    icp_->setGlobalPose(T_pred);

    // 2. LiDAR Corrector step: correct pose using LiDAR scan matching
    icp_->processFrame(current_scan, oxts_data);
    Eigen::Matrix4f icp_pose = icp_->getGlobalPose();

    Eigen::Matrix3f R = icp_pose.block<3,3>(0,0);
    double r = std::atan2(R(2, 1), R(2, 2));
    double p = std::asin(-R(2, 0));
    double y = std::atan2(R(1, 0), R(0, 0));

    Eigen::VectorXd z_icp(6);
    z_icp << icp_pose(0, 3), icp_pose(1, 3), icp_pose(2, 3), r, p, y;
    ekf_.updateICP(z_icp, ekf_icp_noise_);

    // 3. Update with GPS if requested
    if (oxts_data.size() >= 6 && localization_method_ == "EKF_GPS") {
        Eigen::Vector3d z_gps = convertGpsToLocal(oxts_data[0], oxts_data[1], oxts_data[2]);
        ekf_.updateGPS(z_gps, ekf_gps_noise_);
    }

    // 4. Update final global pose from EKF corrected state
    Eigen::VectorXd state = ekf_.getState();
    Eigen::AngleAxisf rollAngle(state(6), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(state(7), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(state(8), Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f R_corrected = (yawAngle * pitchAngle * rollAngle).matrix();

    global_pose_.setIdentity();
    global_pose_.block<3,3>(0,0) = R_corrected;
    global_pose_(0,3) = static_cast<float>(state(0));
    global_pose_(1,3) = static_cast<float>(state(1));
    global_pose_(2,3) = static_cast<float>(state(2));

    icp_->setGlobalPose(global_pose_);
}

void EKFLocalization::savePoseToFile(const std::string& filepath) const {
    std::ofstream file(filepath, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open pose file: " << filepath << std::endl;
        return;
    }
    
    // Save in KITTI format: 3x4 matrix (row-major)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            file << global_pose_(i, j);
            if (!(i == 2 && j == 3)) file << " ";
        }
    }
    file << "\n";
    file.close();
}
