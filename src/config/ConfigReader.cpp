#include "ConfigReader.h"
#include <fstream>
#include <iostream>

ConfigReader::ConfigReader(const std::string& config_path) {
    loadConfig(config_path);
}

void ConfigReader::loadConfig(const std::string& config_path) {
    std::ifstream config_file(config_path);
    
    if (!config_file.is_open()) {
        throw std::runtime_error("Failed to open config file: " + config_path);
    }
    
    try {
        config_file >> config_;
    } catch (const json::exception& e) {
        throw std::runtime_error("Failed to parse JSON config: " + std::string(e.what()));
    }
    
    config_file.close();
}

std::string ConfigReader::getPcdInputPath() const {
    return get<std::string>("PCD_INPUT_PATH");
}

std::string ConfigReader::getPcdOutputPath() const {
    return get<std::string>("PCD_OUTPUT_PATH");
}

std::string ConfigReader::getEstimatedPosesPath() const {
    return get<std::string>("POSES_OUTPUT_PATH");
}

float ConfigReader::getVoxelLeafSize() const {
    return get<float>("VOXEL_LEAF_SIZE");
}

int ConfigReader::getTotalFramesToProcess() const {
    return get<int>("TOTAL_FRAMES_TO_PROCESS");
}

bool ConfigReader::hasKey(const std::string& key) const {
    return config_.contains(key);
}
