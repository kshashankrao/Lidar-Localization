#include "icp.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include <pcl/filters/voxel_grid.h> 
#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h> 

void PointToPlaneICP::filterPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    // The voxel grid filter reduces the number of points in the point cloud by creating a 
    // 3D grid over the data and replacing all points within each voxel with their centroid.
    if (!cloud || cloud->empty()) return;

    pcl::VoxelGrid<pcl::PointXYZI> sor; 
    sor.setInputCloud(cloud);
    // Dimension of the voxel grid (leaf size)
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(*cloud);
}

void PointToPlaneICP::cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    n.setNumberOfThreads(10);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(30);
    n.compute(*normals);
    pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}

void PointToPlaneICP::run(const pcl::PointCloud<pcl::PointXYZI>::Ptr &current_scan, 
                          const pcl::PointCloud<pcl::PointXYZI>::Ptr &previous_scan)
{
    // FIX 1: Allocate as PointXYZ, not PointXYZI
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pcd(new pcl::PointCloud<pcl::PointXYZ>);

    // TODO - Skip the copy by modifying the cloud_with_normal function to take PointXYZI and output PointNormal with intensity field.
    // PCL strips the intensity field automatically during this copy
    pcl::copyPointCloud(*current_scan, *current_pcd);
    pcl::copyPointCloud(*previous_scan, *previous_pcd);

    auto current_scan_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    auto previous_scan_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);

    // TODO - Cache the current normal for next iteration
    cloud_with_normal(current_pcd, current_scan_normals);
    cloud_with_normal(previous_pcd, previous_scan_normals);

    // TODO - Put it in the constructor to initialize it only once 
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> p_icp;
    
    p_icp.setInputSource(current_scan_normals);
    p_icp.setInputTarget(previous_scan_normals);
    p_icp.setTransformationEpsilon(1e-10); 
    p_icp.setMaxCorrespondenceDistance(2.0); // 2.0 meters
    p_icp.setEuclideanFitnessEpsilon(0.001);
    p_icp.setMaximumIterations(35); 
    
    pcl::PointCloud<pcl::PointNormal> aligned_cloud;
    p_icp.align(aligned_cloud);
    
    Eigen::Matrix4f local_transform = p_icp.getFinalTransformation();
    
    global_pose = global_pose * local_transform; 
}

void PointToPlaneICP::savePoseToFile(const std::string& filepath) const {
    std::ofstream file(filepath, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open pose file: " << filepath << std::endl;
        return;
    }
    
    // Save in KITTI format: 3x4 matrix (row-major) - in LiDAR frame
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            file << global_pose(i, j);
            if (!(i == 2 && j == 3)) file << " ";
        }
    }
    file << "\n";
    file.close();
}