#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

class LocalizationBase {
public:
    virtual ~LocalizationBase() = default;

    // Process a single frame.
    // current_scan: The new LiDAR point cloud.
    // oxts_data: The GPS/IMU data for the frame. Can be empty for ICP-only mode.
    virtual void processFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
                              const std::vector<double>& oxts_data) = 0;

    virtual Eigen::Matrix4f getGlobalPose() const = 0;
    virtual void savePoseToFile(const std::string& filepath) const = 0;
};
