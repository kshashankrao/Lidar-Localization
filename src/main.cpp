#include "dataloader/KittiLoader.h"
#include "config/ConfigReader.h"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <memory>
#include "icp/icp.h"
#include "localization/EKFLocalization.h"

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
        } else {
            localization = std::make_unique<PointToPlaneICP>(
                leaf_size, max_correspondence_distance, max_iterations,
                normal_k_search, transformation_epsilon, euclidean_fitness_epsilon
            );
            std::cout << "Using ICP-only Localization." << std::endl;
        }

        int i = 0;

        while (i < total_frames_to_process && loader.hasNext()) 
        {
            std::cout << "Processing frame " << i << "/" << total_frames_to_process << "\r" << std::flush;
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan = loader.getNextCloud();
            std::vector<double> current_oxts = loader.getNextOxts();

            if (!current_scan || current_scan->empty()) 
            {
                continue;
            }

            localization->processFrame(current_scan, current_oxts);
            localization->savePoseToFile(estimated_poses_path);

            i++;
        }

        std::cout << "\nConversion complete! Total files saved: " << i << std::endl;
        return 0;
    
    } 
    
    catch (const std::exception& e) 
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}