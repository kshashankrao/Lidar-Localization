#include "KittiLoader.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>

namespace fs = std::filesystem;

KittiLoader::KittiLoader(const std::string& pcd_dataset_path, const std::string& oxts_dataset_path) 
    : current_index_(0) 
{
    if (!fs::exists(pcd_dataset_path) || !fs::is_directory(pcd_dataset_path)) {
        std::cerr << "Error: Directory does not exist -> " << pcd_dataset_path << std::endl;
        return;
    }

    // 1. Find all .bin files in the PCD directory
    for (const auto& entry : fs::directory_iterator(pcd_dataset_path)) {
        if (entry.path().extension() == ".bin") {
            file_paths_.push_back(entry.path().string());
        }
    }
    std::sort(file_paths_.begin(), file_paths_.end());
    
    // 2. If OXTS path is provided, find all .txt files
    if (!oxts_dataset_path.empty() && fs::exists(oxts_dataset_path) && fs::is_directory(oxts_dataset_path)) {
        for (const auto& entry : fs::directory_iterator(oxts_dataset_path)) {
            if (entry.path().extension() == ".txt") {
                oxts_paths_.push_back(entry.path().string());
            }
        }
        std::sort(oxts_paths_.begin(), oxts_paths_.end());
        
        if (oxts_paths_.size() != file_paths_.size()) {
            std::cerr << "Warning: PCD count (" << file_paths_.size() 
                      << ") does not match OXTS count (" << oxts_paths_.size() << ")" << std::endl;
        }
    }
    
    std::cout << "Successfully loaded " << file_paths_.size() << " frames from " << pcd_dataset_path << std::endl;
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

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::string current_file = file_paths_[current_index_];
    std::ifstream file(current_file, std::ios::binary);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << current_file << std::endl;
        return cloud; 
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    size_t num_points = size / (4 * sizeof(float));
    std::vector<float> buffer(num_points * 4);
    file.read(reinterpret_cast<char*>(buffer.data()), size);
    file.close();

    cloud->points.resize(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        cloud->points[i].x = buffer[i * 4];
        cloud->points[i].y = buffer[i * 4 + 1];
        cloud->points[i].z = buffer[i * 4 + 2];
        cloud->points[i].intensity = buffer[i * 4 + 3];
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

std::vector<double> KittiLoader::getNextOxts() {
    std::vector<double> oxts_data;
    
    if (current_index_ >= oxts_paths_.size()) {
        current_index_++; 
        return oxts_data;
    }

    std::ifstream file(oxts_paths_[current_index_]);
    if (file.is_open()) {
        std::string line;
        if (std::getline(file, line)) {
            std::stringstream ss(line);
            double val;
            while (ss >> val) {
                oxts_data.push_back(val);
            }
        }
        file.close();
    }
    
    current_index_++; // Increment here
    return oxts_data;
}