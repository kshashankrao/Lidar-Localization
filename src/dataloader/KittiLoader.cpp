#include "KittiLoader.h"
#include <iostream>
#include <fstream>
#include <algorithm>

namespace fs = std::filesystem;

KittiLoader::KittiLoader(const std::string& dataset_path) : current_index_(0) {
    if (!fs::exists(dataset_path) || !fs::is_directory(dataset_path)) {
        std::cerr << "Error: Directory does not exist -> " << dataset_path << std::endl;
        return;
    }

    // 1. Find all .bin files in the directory
    for (const auto& entry : fs::directory_iterator(dataset_path)) {
        if (entry.path().extension() == ".bin") {
            file_paths_.push_back(entry.path().string());
        }
    }

    // 2. Sort alphabetically so 000000.bin comes before 000001.bin
    std::sort(file_paths_.begin(), file_paths_.end());
    
    std::cout << "Successfully loaded " << file_paths_.size() << " frames from " << dataset_path << std::endl;
}

bool KittiLoader::hasNext() const {
    return current_index_ < file_paths_.size();
}

size_t KittiLoader::getTotalFrames() const {
    return file_paths_.size();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KittiLoader::getNextCloud() {
    if (!hasNext()) {
        std::cerr << "Warning: No more frames to load." << std::endl;
        return nullptr;
    }

    // Initialize PCL cloud using a smart pointer
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    std::string current_file = file_paths_[current_index_];
    std::ifstream file(current_file, std::ios::binary);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << current_file << std::endl;
        current_index_++;
        return cloud; // Return empty cloud on failure
    }

    // Find file size to pre-allocate memory
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    // KITTI stores data as float32: (x, y, z, intensity) -> 4 floats = 16 bytes per point
    size_t num_points = size / (4 * sizeof(float));
    
    // Read entire binary file into a raw buffer efficiently
    std::vector<float> buffer(num_points * 4);
    file.read(reinterpret_cast<char*>(buffer.data()), size);
    file.close();

    // Resize PCL cloud and map the data
    cloud->points.resize(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        cloud->points[i].x = buffer[i * 4];
        cloud->points[i].y = buffer[i * 4 + 1];
        cloud->points[i].z = buffer[i * 4 + 2];
        cloud->points[i].intensity = buffer[i * 4 + 3];
    }

    // Standard PCL metadata
    cloud->width = cloud->points.size();
    cloud->height = 1; // Unorganized point cloud
    cloud->is_dense = true;

    current_index_++;
    return cloud;
}