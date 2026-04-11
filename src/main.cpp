#include "dataloader/KittiLoader.h"
#include "config/ConfigReader.h"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include "icp/icp.h"

namespace fs = std::filesystem;

int main() 
{
    try 
    {
        // Load configuration from JSON file
        ConfigReader config("config/config.json");
        
        std::string data_path = config.getPcdInputPath();
        std::string output_path = config.getPcdOutputPath();
        std::string estimated_poses_path = config.getEstimatedPosesPath();
        int total_frames_to_process = config.getTotalFramesToProcess();
        float leaf_size = config.getVoxelLeafSize();
        
        KittiLoader loader(data_path);

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

        PointToPlaneICP icp(leaf_size);

        int i = 0;

        pcl::PointCloud<pcl::PointXYZI>::Ptr previous_scan = nullptr;
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan = nullptr;
        previous_scan = loader.getNextCloud();
        icp.filterPointCloud(previous_scan);

        while (i < total_frames_to_process && loader.hasNext()) 
        {
            std::cout << "Processing frame " << i << "/" << total_frames_to_process << "\r" << std::flush;
            current_scan = loader.getNextCloud();
            icp.filterPointCloud(current_scan);

            if (!current_scan || current_scan->empty() || !previous_scan || previous_scan->empty()) 
            {
                continue;
            }

            icp.run(current_scan, previous_scan);
            icp.savePoseToFile(estimated_poses_path);

            // Save the point cloud
            // std::stringstream ss;
            // ss << output_path << "/frame_" << std::setfill('0') << std::setw(4) << i << ".pcd";
            // std::string filename = ss.str();

            // if (pcl::io::savePCDFileBinary(filename, *current_scan) == 0) 
            // {
            //     if (i % 10 == 0) { // Log every 10 frames to keep the console clean
            //         std::cout << "Saved: " << filename << std::endl;
            //     }
            // } 
            // else 
            // {
            //     std::cerr << "Failed to save: " << filename << std::endl;
            // }
            previous_scan = current_scan;
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