#pragma once
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include "../localization/LocalizationBase.h"

class PointToPlaneICP : public LocalizationBase
{
    private:
        float leaf_size_;
        float max_correspondence_distance_;
        int   max_iterations_;
        int   normal_k_search_;
        double transformation_epsilon_;
        double euclidean_fitness_epsilon_;

        Eigen::Matrix4f global_pose;
        pcl::PointCloud<pcl::PointXYZI>::Ptr previous_scan_;

    public:
        PointToPlaneICP(float leaf_size,
                        float max_correspondence_distance = 2.0f,
                        int   max_iterations              = 35,
                        int   normal_k_search             = 30,
                        double transformation_epsilon     = 1e-10,
                        double euclidean_fitness_epsilon  = 0.001)
            : leaf_size_(leaf_size),
              max_correspondence_distance_(max_correspondence_distance),
              max_iterations_(max_iterations),
              normal_k_search_(normal_k_search),
              transformation_epsilon_(transformation_epsilon),
              euclidean_fitness_epsilon_(euclidean_fitness_epsilon),
              global_pose(Eigen::Matrix4f::Identity()),
              previous_scan_(nullptr)
        {
        }

        void filterPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
        void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals);

        void processFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr &current_scan, 
                          const std::vector<double>& oxts_data) override;
        
        Eigen::Matrix4f getGlobalPose() const override { return global_pose; }
        void setGlobalPose(const Eigen::Matrix4f& pose) { global_pose = pose; }
        void resetPose() { global_pose = Eigen::Matrix4f::Identity(); }
        
        void savePoseToFile(const std::string& filepath) const override;
};