#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class ConfigReader {
public:
    ConfigReader(const std::string& config_path);
    
    // Getters for configuration values
    std::string getPcdInputPath() const;
    std::string getPcdOutputPath() const;
    std::string getEstimatedPosesPath() const;
    int getTotalFramesToProcess() const;
    float getVoxelLeafSize() const;
    
    // Generic getter for any JSON value
    template<typename T>
    T get(const std::string& key) const;
    
    // Check if key exists
    bool hasKey(const std::string& key) const;
    
private:
    json config_;
    void loadConfig(const std::string& config_path);
};

// Template implementation
template<typename T>
T ConfigReader::get(const std::string& key) const {
    if (config_.contains(key)) {
        return config_[key].get<T>();
    }
    throw std::runtime_error("Config key not found: " + key);
}

#endif // CONFIG_READER_H
