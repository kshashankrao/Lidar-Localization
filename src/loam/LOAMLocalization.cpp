#include "LOAMLocalization.h"
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <Eigen/Eigenvalues>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <omp.h>

LOAMLocalization::LOAMLocalization(float corner_leaf_size,
                                   float surf_leaf_size,
                                   float map_leaf_size,
                                   float max_correspondence_dist,
                                   int   max_iterations,
                                   float loop_search_radius,
                                   int   loop_history_gap)
    : corner_leaf_size_(corner_leaf_size),
      surf_leaf_size_(surf_leaf_size),
      map_leaf_size_(map_leaf_size),
      max_correspondence_dist_(max_correspondence_dist),
      max_iterations_(max_iterations),
      loop_search_radius_(loop_search_radius),
      loop_history_gap_(loop_history_gap),
      global_pose_(Eigen::Matrix4f::Identity()),
      last_keyframe_pose_(Eigen::Matrix4f::Identity()),
      frame_count_(0),
      local_map_corner_(new pcl::PointCloud<pcl::PointXYZI>()),
      local_map_surf_(new pcl::PointCloud<pcl::PointXYZI>()),
      kdtree_local_corner_(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
      kdtree_local_surf_(new pcl::KdTreeFLANN<pcl::PointXYZI>())
{
    max_iterations_ = std::min(max_iterations_, 12);
}

void LOAMLocalization::extractFeatures(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr& corner_cloud,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_cloud)
{
    corner_cloud->clear();
    surf_cloud->clear();

    if (!input_cloud || input_cloud->size() < 20) return;

    // Filter valid range points
    pcl::PointCloud<pcl::PointXYZI>::Ptr valid_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    valid_cloud->reserve(input_cloud->size());
    for (const auto& pt : input_cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (r2 < 3.0f || r2 > 85.0f * 85.0f) continue;
        valid_cloud->push_back(pt);
    }

    int cloud_size = static_cast<int>(valid_cloud->size());
    if (cloud_size < 20) return;

    std::vector<float> cloud_curvature(cloud_size, 0.0f);
    std::vector<int> cloud_sort_ind(cloud_size, 0);
    std::vector<int> cloud_neighbor_picked(cloud_size, 0);

    for (int i = 5; i < cloud_size - 5; ++i) {
        float diffX = 0, diffY = 0, diffZ = 0;
        for (int j = -5; j <= 5; ++j) {
            if (j == 0) continue;
            diffX += valid_cloud->points[i + j].x - valid_cloud->points[i].x;
            diffY += valid_cloud->points[i + j].y - valid_cloud->points[i].y;
            diffZ += valid_cloud->points[i + j].z - valid_cloud->points[i].z;
        }
        cloud_curvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloud_sort_ind[i] = i;
    }

    // Divide cloud into 6 sectors for even feature distribution
    int num_sectors = 6;
    for (int sec = 0; sec < num_sectors; ++sec) {
        int sp = 5 + (cloud_size - 10) * sec / num_sectors;
        int ep = 5 + (cloud_size - 10) * (sec + 1) / num_sectors - 1;
        if (sp >= ep) continue;

        std::sort(cloud_sort_ind.begin() + sp, cloud_sort_ind.begin() + ep + 1,
                  [&cloud_curvature](int i1, int i2) {
                      return cloud_curvature[i1] < cloud_curvature[i2];
                  });

        // Pick top corners
        int largest_picked = 0;
        for (int k = ep; k >= sp; --k) {
            int ind = cloud_sort_ind[k];
            if (cloud_neighbor_picked[ind] == 0 && cloud_curvature[ind] > 0.5f) {
                largest_picked++;
                if (largest_picked <= 30) {
                    corner_cloud->push_back(valid_cloud->points[ind]);
                    cloud_neighbor_picked[ind] = 1;
                    for (int l = 1; l <= 5; ++l) {
                        if (ind + l < cloud_size) cloud_neighbor_picked[ind + l] = 1;
                        if (ind - l >= 0) cloud_neighbor_picked[ind - l] = 1;
                    }
                }
            }
        }

        // Pick surf points
        for (int k = sp; k <= ep; ++k) {
            int ind = cloud_sort_ind[k];
            if (cloud_neighbor_picked[ind] == 0 && cloud_curvature[ind] < 0.15f) {
                surf_cloud->push_back(valid_cloud->points[ind]);
                cloud_neighbor_picked[ind] = 1;
                for (int l = 1; l <= 3; ++l) {
                    if (ind + l < cloud_size) cloud_neighbor_picked[ind + l] = 1;
                    if (ind - l >= 0) cloud_neighbor_picked[ind - l] = 1;
                }
            }
        }
    }

    // VoxelGrid downsample extracted features
    if (!corner_cloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(corner_cloud);
        vg.setLeafSize(corner_leaf_size_, corner_leaf_size_, corner_leaf_size_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
        vg.filter(*tmp);
        corner_cloud = tmp;
    }
    if (!surf_cloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(surf_cloud);
        vg.setLeafSize(surf_leaf_size_, surf_leaf_size_, surf_leaf_size_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
        vg.filter(*tmp);
        surf_cloud = tmp;
    }
}

void LOAMLocalization::scanToMapOptimization(const pcl::PointCloud<pcl::PointXYZI>::Ptr& corner_cloud,
                                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_cloud)
{
    if (local_map_corner_->size() < 50 || local_map_surf_->size() < 100) {
        return;
    }

    kdtree_local_corner_->setInputCloud(local_map_corner_);
    kdtree_local_surf_->setInputCloud(local_map_surf_);

    // Optimize pose over max_iterations_
    for (int iter = 0; iter < max_iterations_; ++iter) {
        Eigen::Matrix3f R_cur = global_pose_.block<3, 3>(0, 0);
        Eigen::Vector3f t_cur = global_pose_.block<3, 1>(0, 3);
        Eigen::Vector3d t_cur_d(t_cur.x(), t_cur.y(), t_cur.z());

        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

        int num_residuals = 0;

        // 1. Point-to-Line (Corner) residuals
        for (const auto& pt : corner_cloud->points) {
            Eigen::Vector3f p_local(pt.x, pt.y, pt.z);
            Eigen::Vector3f p_global = R_cur * p_local + t_cur;
            pcl::PointXYZI p_query;
            p_query.x = p_global.x();
            p_query.y = p_global.y();
            p_query.z = p_global.z();

            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            if (kdtree_local_corner_->nearestKSearch(p_query, 5, pointSearchInd, pointSearchSqDis) == 5) {
                if (pointSearchSqDis[4] < max_correspondence_dist_ * max_correspondence_dist_) {
                    Eigen::Vector3d center(0, 0, 0);
                    for (int j = 0; j < 5; ++j) {
                        const auto& np = local_map_corner_->points[pointSearchInd[j]];
                        center += Eigen::Vector3d(np.x, np.y, np.z);
                    }
                    center /= 5.0;

                    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                    for (int j = 0; j < 5; ++j) {
                        const auto& np = local_map_corner_->points[pointSearchInd[j]];
                        Eigen::Vector3d diff = Eigen::Vector3d(np.x, np.y, np.z) - center;
                        cov += diff * diff.transpose();
                    }
                    cov /= 5.0;

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);
                    if (saes.eigenvalues()[2] > 3.0 * saes.eigenvalues()[1]) {
                        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                        Eigen::Vector3d pg(p_global.x(), p_global.y(), p_global.z());
                        Eigen::Vector3d diff = pg - center;
                        Eigen::Vector3d ortho_vec = diff - unit_direction * (unit_direction.dot(diff));
                        double res = ortho_vec.norm();

                        if (res < max_correspondence_dist_) {
                            Eigen::Vector3d w_dir = Eigen::Vector3d::Zero();
                            if (res > 1e-6) w_dir = ortho_vec / res;
                            Eigen::Matrix<double, 1, 6> J;
                            J.block<1, 3>(0, 0) = w_dir.transpose();
                            J.block<1, 3>(0, 3) = ((pg - t_cur_d).cross(w_dir)).transpose();

                            double weight = (res < 0.2) ? 1.0 : (0.2 / res);
                            H += weight * J.transpose() * J;
                            b += weight * J.transpose() * res;
                            num_residuals++;
                        }
                    }
                }
            }
        }

        // 2. Point-to-Plane (Surf) residuals
        for (const auto& pt : surf_cloud->points) {
            Eigen::Vector3f p_local(pt.x, pt.y, pt.z);
            Eigen::Vector3f p_global = R_cur * p_local + t_cur;
            pcl::PointXYZI p_query;
            p_query.x = p_global.x();
            p_query.y = p_global.y();
            p_query.z = p_global.z();

            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            if (kdtree_local_surf_->nearestKSearch(p_query, 5, pointSearchInd, pointSearchSqDis) == 5) {
                if (pointSearchSqDis[4] < max_correspondence_dist_ * max_correspondence_dist_) {
                    Eigen::Matrix<double, 5, 3> A;
                    Eigen::Matrix<double, 5, 1> B_vec;
                    for (int j = 0; j < 5; ++j) {
                        const auto& np = local_map_surf_->points[pointSearchInd[j]];
                        A(j, 0) = np.x;
                        A(j, 1) = np.y;
                        A(j, 2) = np.z;
                        B_vec(j, 0) = -1.0;
                    }

                    // Solve for plane normal A * n = -1 using exact Householder QR
                    Eigen::Vector3d norm_vec = A.householderQr().solve(B_vec);
                    double norm_len = norm_vec.norm();

                    if (norm_len > 1e-6) {
                        Eigen::Vector3d unit_norm = norm_vec / norm_len;
                        double d = 1.0 / norm_len;

                        bool valid_plane = true;
                        for (int j = 0; j < 5; ++j) {
                            Eigen::Vector3d pt_j(local_map_surf_->points[pointSearchInd[j]].x,
                                                 local_map_surf_->points[pointSearchInd[j]].y,
                                                 local_map_surf_->points[pointSearchInd[j]].z);
                            if (std::abs(unit_norm.dot(pt_j) + d) > 0.15) {
                                valid_plane = false;
                                break;
                            }
                        }
                        if (valid_plane) {
                            Eigen::Vector3d pg(p_global.x(), p_global.y(), p_global.z());
                            double res = unit_norm.dot(pg) + d;
                            if (std::abs(res) < max_correspondence_dist_) {
                                Eigen::Matrix<double, 1, 6> J;
                                J.block<1, 3>(0, 0) = unit_norm.transpose();
                                J.block<1, 3>(0, 3) = ((pg - t_cur_d).cross(unit_norm)).transpose();

                                double weight = (std::abs(res) < 0.2) ? 1.0 : (0.2 / std::abs(res));
                                H += weight * J.transpose() * J;
                                b += weight * J.transpose() * res;
                                num_residuals++;
                            }
                        }
                    }
                }
            }
        }

        if (num_residuals < 20) break;

        // Add Levenberg-Marquardt damping
        H += Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
        Eigen::Matrix<double, 6, 1> delta_xi = -H.ldlt().solve(b);

        if (std::isnan(delta_xi(0))) break;

        // Update translation
        global_pose_(0, 3) += static_cast<float>(delta_xi(0));
        global_pose_(1, 3) += static_cast<float>(delta_xi(1));
        global_pose_(2, 3) += static_cast<float>(delta_xi(2));

        // Update rotation
        Eigen::Vector3f d_omega(static_cast<float>(delta_xi(3)),
                                static_cast<float>(delta_xi(4)),
                                static_cast<float>(delta_xi(5)));
        float angle = d_omega.norm();
        if (angle > 1e-6f) {
            Eigen::AngleAxisf aa(angle, d_omega / angle);
            global_pose_.block<3, 3>(0, 0) = aa.toRotationMatrix() * global_pose_.block<3, 3>(0, 0);
        }

        if (delta_xi.norm() < 1e-4) break;
    }
}

bool LOAMLocalization::detectAndCorrectLoopClosure()
{
    if (keyframes_.size() < static_cast<size_t>(loop_history_gap_ + 5)) {
        return false;
    }

    int cur_idx = static_cast<int>(keyframes_.size()) - 1;
    Eigen::Vector3f cur_pos = keyframes_[cur_idx].pose.block<3, 1>(0, 3);

    int best_match_idx = -1;
    float min_dist = loop_search_radius_;

    for (int i = 0; i <= cur_idx - loop_history_gap_; ++i) {
        Eigen::Vector3f hist_pos = keyframes_[i].pose.block<3, 1>(0, 3);
        float dist = (cur_pos - hist_pos).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_match_idx = i;
        }
    }

    if (best_match_idx == -1) return false;

    // Assemble historical local target map around best_match_idx
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (int idx = std::max(0, best_match_idx - 3);
         idx <= std::min(static_cast<int>(keyframes_.size()) - 1, best_match_idx + 3);
         ++idx) {
        pcl::PointCloud<pcl::PointXYZI> trans_corner, trans_surf;
        pcl::transformPointCloud(*keyframes_[idx].corner_cloud, trans_corner, keyframes_[idx].pose);
        pcl::transformPointCloud(*keyframes_[idx].surf_cloud, trans_surf, keyframes_[idx].pose);
        *target_cloud += trans_corner;
        *target_cloud += trans_surf;
    }

    // Assemble source cloud from current keyframe
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI> cur_trans_corner, cur_trans_surf;
    pcl::transformPointCloud(*keyframes_[cur_idx].corner_cloud, cur_trans_corner, keyframes_[cur_idx].pose);
    pcl::transformPointCloud(*keyframes_[cur_idx].surf_cloud, cur_trans_surf, keyframes_[cur_idx].pose);
    *source_cloud += cur_trans_corner;
    *source_cloud += cur_trans_surf;

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(max_correspondence_dist_);
    icp.setMaximumIterations(35);
    icp.setTransformationEpsilon(1e-6);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    pcl::PointCloud<pcl::PointXYZI> unused_aligned;
    icp.align(unused_aligned);

    if (icp.hasConverged() && icp.getFitnessScore() < 0.025) {
        Eigen::Matrix4f correction = icp.getFinalTransformation();
        if (correction.block<3, 1>(0, 3).norm() > 0.5f) {
            return false;
        }

        int start_f = keyframes_[best_match_idx].frame_index;
        int end_f   = keyframes_[cur_idx].frame_index;
        int loop_len_f = end_f - start_f;
        if (loop_len_f <= 0) return false;

        Eigen::AngleAxisf aa_corr(correction.block<3, 3>(0, 0));

        // Apply drift correction backward to keyframes along the loop
        for (int i = best_match_idx + 1; i <= cur_idx; ++i) {
            float ratio = static_cast<float>(keyframes_[i].frame_index - start_f) / static_cast<float>(loop_len_f);
            Eigen::Vector3f t_corr = correction.block<3, 1>(0, 3) * ratio;
            Eigen::AngleAxisf aa_ratio(aa_corr.angle() * ratio, aa_corr.axis());
            keyframes_[i].pose.block<3, 1>(0, 3) += t_corr;
            keyframes_[i].pose.block<3, 3>(0, 0) = aa_ratio.toRotationMatrix() * keyframes_[i].pose.block<3, 3>(0, 0);
        }

        // Continuously correct ALL frames in pose_history_
        for (int f = start_f + 1; f < static_cast<int>(pose_history_.size()); ++f) {
            float ratio = std::min(1.0f, static_cast<float>(f - start_f) / static_cast<float>(loop_len_f));
            Eigen::Vector3f t_corr = correction.block<3, 1>(0, 3) * ratio;
            Eigen::AngleAxisf aa_ratio(aa_corr.angle() * ratio, aa_corr.axis());
            pose_history_[f].block<3, 1>(0, 3) += t_corr;
            pose_history_[f].block<3, 3>(0, 0) = aa_ratio.toRotationMatrix() * pose_history_[f].block<3, 3>(0, 0);
        }

        global_pose_ = keyframes_[cur_idx].pose;

        // Rebuild local map from recent corrected keyframes
        local_map_corner_->clear();
        local_map_surf_->clear();
        int start_idx = std::max(0, static_cast<int>(keyframes_.size()) - 30);
        for (size_t k = start_idx; k < keyframes_.size(); ++k) {
            pcl::PointCloud<pcl::PointXYZI> c_tr, s_tr;
            pcl::transformPointCloud(*keyframes_[k].corner_cloud, c_tr, keyframes_[k].pose);
            pcl::transformPointCloud(*keyframes_[k].surf_cloud, s_tr, keyframes_[k].pose);
            *local_map_corner_ += c_tr;
            *local_map_surf_ += s_tr;
        }

        return true;
    }

    return false;
}

void LOAMLocalization::updateLocalMap()
{
    // Check keyframe criterion
    float dist_from_kf = (global_pose_.block<3, 1>(0, 3) - last_keyframe_pose_.block<3, 1>(0, 3)).norm();
    if (keyframes_.empty() || dist_from_kf > 0.7f) {
        LOAMKeyframe kf;
        kf.frame_index = frame_count_;
        kf.pose = global_pose_;
        kf.corner_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        kf.surf_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        // Store copy of features extracted in processFrame
        last_keyframe_pose_ = global_pose_;
    }
}

void LOAMLocalization::processFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
                                    const std::vector<double>& oxts_data)
{
    // 1. Predict initial pose using constant velocity model (Pure LiDAR Odometry without GPS/external aiding)
    if (pose_history_.size() >= 2) {
        Eigen::Matrix4f delta = pose_history_[pose_history_.size() - 2].inverse() * pose_history_.back();
        global_pose_ = pose_history_.back() * delta;
    }

    // 2. Extract corner and surf features
    pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    extractFeatures(current_scan, corner_cloud, surf_cloud);

    // 3. Scan-to-Map optimization
    scanToMapOptimization(corner_cloud, surf_cloud);

    // 4. Keyframe update & Local Map maintenance
    float dist_from_kf = (global_pose_.block<3, 1>(0, 3) - last_keyframe_pose_.block<3, 1>(0, 3)).norm();
    if (keyframes_.empty() || dist_from_kf > 0.6f) {
        LOAMKeyframe kf;
        kf.frame_index = frame_count_;
        kf.pose = global_pose_;
        kf.corner_cloud = corner_cloud;
        kf.surf_cloud = surf_cloud;
        keyframes_.push_back(kf);
        last_keyframe_pose_ = global_pose_;

        // Build local map from last 25 keyframes (~15m sliding window for sharper foliage map)
        local_map_corner_->clear();
        local_map_surf_->clear();
        size_t start_k = keyframes_.size() > 25 ? keyframes_.size() - 25 : 0;
        for (size_t k = start_k; k < keyframes_.size(); ++k) {
            pcl::PointCloud<pcl::PointXYZI> c_tr, s_tr;
            pcl::transformPointCloud(*keyframes_[k].corner_cloud, c_tr, keyframes_[k].pose);
            pcl::transformPointCloud(*keyframes_[k].surf_cloud, s_tr, keyframes_[k].pose);
            *local_map_corner_ += c_tr;
            *local_map_surf_ += s_tr;
        }

        // Downsample local map periodically
        if (local_map_corner_->size() > 5000) {
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setInputCloud(local_map_corner_);
            vg.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
            vg.filter(*tmp);
            local_map_corner_ = tmp;
        }
        if (local_map_surf_->size() > 15000) {
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setInputCloud(local_map_surf_);
            vg.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
            vg.filter(*tmp);
            local_map_surf_ = tmp;
        }

        // 5. Detect and correct loop closure
        detectAndCorrectLoopClosure();
    }

    pose_history_.push_back(global_pose_);
    frame_count_++;
}

void LOAMLocalization::savePoseToFile(const std::string& filepath) const
{
    // Overwrite file with all trajectory poses so loop closure corrections are saved
    std::ofstream file(filepath, std::ios::trunc);
    if (!file.is_open()) {
        std::cerr << "Failed to open pose file: " << filepath << std::endl;
        return;
    }

    for (const auto& pose : pose_history_) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                file << pose(i, j);
                if (!(i == 2 && j == 3)) file << " ";
            }
        }
        file << "\n";
    }
    file.close();
}
