#pragma once

#include "../localization/LocalizationBase.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>

struct LOAMKeyframe {
    int frame_index;
    Eigen::Matrix4f pose;
    pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_cloud;
};

class LOAMLocalization : public LocalizationBase {
public:
    LOAMLocalization(float corner_leaf_size = 0.2f,
                     float surf_leaf_size   = 0.4f,
                     float map_leaf_size    = 0.4f,
                     float max_correspondence_dist = 2.0f,
                     int   max_iterations          = 4,
                     float loop_search_radius      = 4.0f,
                     int   loop_history_gap        = 300);

    void processFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
                      const std::vector<double>& oxts_data) override;

    Eigen::Matrix4f getGlobalPose() const override { return global_pose_; }
    void savePoseToFile(const std::string& filepath) const override;

private:
    void extractFeatures(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& corner_cloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_cloud);

    void scanToMapOptimization(const pcl::PointCloud<pcl::PointXYZI>::Ptr& corner_cloud,
                               const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_cloud);

    void updateLocalMap();
    bool detectAndCorrectLoopClosure();

    // Hyperparameters
    float corner_leaf_size_;
    float surf_leaf_size_;
    float map_leaf_size_;
    float max_correspondence_dist_;
    int   max_iterations_;
    float loop_search_radius_;
    int   loop_history_gap_;

    // Poses & Trajectory history
    Eigen::Matrix4f global_pose_;
    Eigen::Matrix4f last_keyframe_pose_;
    std::vector<Eigen::Matrix4f> pose_history_;

    // Keyframes
    std::vector<LOAMKeyframe> keyframes_;
    int frame_count_;

    // Local Map
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_corner_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_surf_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_local_corner_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_local_surf_;
};
