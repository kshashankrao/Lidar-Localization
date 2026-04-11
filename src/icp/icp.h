#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

class PointToPlaneICP
{
    private:
        float leaf_size_;
        Eigen::Matrix4f global_pose;

    public:
        PointToPlaneICP(float leaf_size) : leaf_size_(leaf_size), global_pose(Eigen::Matrix4f::Identity()) 
        {
        }

        void filterPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
        void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals);

        void run(const pcl::PointCloud<pcl::PointXYZI>::Ptr &current_scan, 
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr &previous_scan);
        
        Eigen::Matrix4f getGlobalPose() const { return global_pose; }
        void setGlobalPose(const Eigen::Matrix4f& pose) { global_pose = pose; }
        void resetPose() { global_pose = Eigen::Matrix4f::Identity(); }
        
        void savePoseToFile(const std::string& filepath) const;
};