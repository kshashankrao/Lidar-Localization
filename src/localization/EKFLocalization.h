#pragma once

#include "LocalizationBase.h"
#include "../icp/icp.h"
#include "../ekf/ekf.h"
#include <memory>
#include <random>

class EKFLocalization : public LocalizationBase {
public:
    EKFLocalization(float leaf_size,
                    float max_correspondence_distance,
                    int   max_iterations,
                    int   normal_k_search,
                    double transformation_epsilon,
                    double euclidean_fitness_epsilon,
                    double ekf_process_noise,
                    double ekf_gps_noise,
                    double ekf_icp_noise,
                    double ekf_imu_noise,
                    const std::string& localization_method);

    void processFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
                      const std::vector<double>& oxts_data) override;

    Eigen::Matrix4f getGlobalPose() const override { return global_pose_; }
    void savePoseToFile(const std::string& filepath) const override;

private:
    std::unique_ptr<PointToPlaneICP> icp_;
    EKF ekf_;
    
    double ekf_process_noise_;
    double ekf_gps_noise_;
    double ekf_icp_noise_;
    double ekf_imu_noise_;
    std::string localization_method_;
    
    Eigen::Matrix4f global_pose_;

    // For GPS to local coordinate conversion
    bool first_gps_received_;
    double lat_0_, lon_0_, alt_0_;
    double initial_yaw_;
    Eigen::Matrix2d R_align_;

    Eigen::Vector3d convertGpsToLocal(double lat, double lon, double alt);

    // Random generators for synthetic GPS noise
    std::mt19937 random_generator_;
    std::normal_distribution<double> noise_dist_xy_;
    std::normal_distribution<double> noise_dist_z_;
    std::normal_distribution<double> noise_dist_rpy_;
};
