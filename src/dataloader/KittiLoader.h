#pragma once

#include <string>
#include <vector>
#include <filesystem>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class KittiLoader {
public:
    // Constructor takes the path to the 'velodyne_points/data' directory
    explicit KittiLoader(const std::string& dataset_path);

    // Checks if there are more frames to process
    bool hasNext() const;

    // Loads and returns the next point cloud as a PCL shared pointer
    pcl::PointCloud<pcl::PointXYZI>::Ptr getNextCloud();

    // Returns the total number of frames found
    size_t getTotalFrames() const;

private:
    std::vector<std::string> file_paths_;
    size_t current_index_;
};