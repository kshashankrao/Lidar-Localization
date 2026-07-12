#include "dataloader/KittiLoader.h"
#include "config/ConfigReader.h"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <memory>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <cmath>
#include "icp/icp.h"
#include "localization/EKFLocalization.h"
#include "loam/LOAMLocalization.h"

namespace fs = std::filesystem;

int main(int argc, char* argv[]) 
{
    try 
    {
        // Config path can be overridden via argv[1] (used by Optuna tuner per-trial)
        std::string config_path = (argc > 1) ? argv[1] : "config/config.json";
        ConfigReader config(config_path);
        
        std::string data_path = config.getPcdInputPath();
        std::string output_path = config.getPcdOutputPath();
        std::string estimated_poses_path = config.getEstimatedPosesPath();
        int total_frames_to_process = config.getTotalFramesToProcess();
        float leaf_size = config.getVoxelLeafSize();

        // ICP hyperparameters — tunable via config.json (and Optuna)
        float  max_correspondence_distance = config.get<float>("MAX_CORRESPONDENCE_DISTANCE");
        int    max_iterations              = config.get<int>("MAX_ITERATIONS");
        int    normal_k_search             = config.get<int>("NORMAL_K_SEARCH");
        double transformation_epsilon      = config.get<double>("TRANSFORMATION_EPSILON");
        double euclidean_fitness_epsilon   = config.get<double>("EUCLIDEAN_FITNESS_EPSILON");
        
        // EKF & Localization params
        std::string localization_method = "ICP";
        if (config.hasKey("LOCALIZATION_METHOD")) {
            localization_method = config.get<std::string>("LOCALIZATION_METHOD");
        }
        std::string oxts_path = "";
        if (config.hasKey("OXTS_INPUT_PATH")) {
            oxts_path = config.get<std::string>("OXTS_INPUT_PATH");
        }

        KittiLoader loader(data_path, oxts_path);

        if (loader.getTotalFrames() == 0) 
        {
            std::cerr << "Error: No KITTI frames found at the specified path." << std::endl;
            return -1;
        }

        if (!fs::exists(output_path)) 
        {
            fs::create_directories(output_path);
            std::cout << "Created output directory: " << output_path << std::endl;
        }

        // Clear the estimated poses file and prepare for writing
        std::ofstream poses_file(estimated_poses_path);
        poses_file.close();
        std::cout << "Initialized poses file: " << estimated_poses_path << std::endl;

        std::unique_ptr<LocalizationBase> localization;
        
        if (localization_method == "EKF_GPS" || localization_method == "EKF_ICP" || localization_method == "EKF_IMU") {
            double ekf_proc_noise = config.get<double>("EKF_PROCESS_NOISE");
            double ekf_gps_noise = config.get<double>("EKF_GPS_NOISE");
            double ekf_icp_noise = config.get<double>("EKF_ICP_NOISE");
            
            localization = std::make_unique<EKFLocalization>(
                leaf_size, max_correspondence_distance, max_iterations,
                normal_k_search, transformation_epsilon, euclidean_fitness_epsilon,
                ekf_proc_noise, ekf_gps_noise, ekf_icp_noise, localization_method
            );
            std::cout << "Using " << localization_method << " Localization." << std::endl;
        } else if (localization_method == "LOAM") {
            float corner_leaf_size = config.hasKey("LOAM_CORNER_LEAF_SIZE") ? config.get<float>("LOAM_CORNER_LEAF_SIZE") : 0.2f;
            float surf_leaf_size   = config.hasKey("LOAM_SURF_LEAF_SIZE") ? config.get<float>("LOAM_SURF_LEAF_SIZE") : 0.4f;
            float map_leaf_size    = config.hasKey("LOAM_MAP_LEAF_SIZE") ? config.get<float>("LOAM_MAP_LEAF_SIZE") : 0.4f;
            localization = std::make_unique<LOAMLocalization>(
                corner_leaf_size, surf_leaf_size, map_leaf_size,
                max_correspondence_distance, max_iterations
            );
            std::cout << "Using LOAM with Loop Closure Localization." << std::endl;
        } else {
            localization = std::make_unique<PointToPlaneICP>(
                leaf_size, max_correspondence_distance, max_iterations,
                normal_k_search, transformation_epsilon, euclidean_fitness_epsilon
            );
            std::cout << "Using ICP-only Localization." << std::endl;
        }

        int i = 0;
        std::vector<double> frame_times_ms;
        frame_times_ms.reserve(total_frames_to_process);

        while (i < total_frames_to_process && loader.hasNext()) 
        {
            std::cout << "Processing frame " << i << "/" << total_frames_to_process << "\r" << std::flush;
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan = loader.getNextCloud();
            std::vector<double> current_oxts = loader.getNextOxts();

            if (!current_scan || current_scan->empty()) 
            {
                continue;
            }

            auto start_t = std::chrono::high_resolution_clock::now();
            localization->processFrame(current_scan, current_oxts);
            auto end_t = std::chrono::high_resolution_clock::now();
            double duration_ms = std::chrono::duration<double, std::milli>(end_t - start_t).count();
            frame_times_ms.push_back(duration_ms);

            localization->savePoseToFile(estimated_poses_path);

            i++;
        }

        std::cout << "\nConversion complete! Total files saved: " << i << std::endl;

        if (!frame_times_ms.empty()) {
            double total_ms = std::accumulate(frame_times_ms.begin(), frame_times_ms.end(), 0.0);
            double mean_ms = total_ms / frame_times_ms.size();
            double sq_sum = 0.0;
            for (double t : frame_times_ms) {
                sq_sum += (t - mean_ms) * (t - mean_ms);
            }
            double stddev_ms = std::sqrt(sq_sum / frame_times_ms.size());

            std::vector<double> sorted_times = frame_times_ms;
            std::sort(sorted_times.begin(), sorted_times.end());
            double min_ms = sorted_times.front();
            double max_ms = sorted_times.back();
            double p50_ms = sorted_times[static_cast<size_t>(sorted_times.size() * 0.50)];
            double p90_ms = sorted_times[static_cast<size_t>(sorted_times.size() * 0.90)];
            double p99_ms = sorted_times[static_cast<size_t>(sorted_times.size() * 0.99)];
            double fps = 1000.0 / mean_ms;

            std::cout << "\n============================================================" << std::endl;
            std::cout << "               RUNTIME PERFORMANCE SUMMARY                  " << std::endl;
            std::cout << "============================================================" << std::endl;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Total Frames Processed : " << frame_times_ms.size() << std::endl;
            std::cout << "Mean Runtime per Frame : " << mean_ms << " ms  (" << fps << " FPS)" << std::endl;
            std::cout << "Std Dev                : " << stddev_ms << " ms" << std::endl;
            std::cout << "Min / Max Runtime      : " << min_ms << " ms / " << max_ms << " ms" << std::endl;
            std::cout << "Median (50th %ile)     : " << p50_ms << " ms" << std::endl;
            std::cout << "90th %ile Runtime      : " << p90_ms << " ms" << std::endl;
            std::cout << "99th %ile Runtime      : " << p99_ms << " ms" << std::endl;
            std::cout << "============================================================" << std::endl;
        }
        return 0;
    
    } 
    
    catch (const std::exception& e) 
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}