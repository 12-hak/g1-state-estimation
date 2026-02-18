#pragma once

#include "g1_slam/voxel_hash_map.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace g1_slam {

struct KissICPConfig {
    double voxel_size = 0.5;
    double max_range = 15.0;
    double min_range = 0.3;
    int max_iterations = 50;
    double convergence_threshold = 1e-4;
    double initial_threshold = 2.0;   // Adaptive threshold start
    double min_motion_threshold = 0.1;
};

struct ICPResult {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    bool converged = false;
    double fitness_score = 0.0;
    int iterations = 0;
};

class KissICPWrapper {
public:
    explicit KissICPWrapper(const KissICPConfig& config = KissICPConfig());

    ICPResult registerFrame(const std::vector<Eigen::Vector3d>& frame,
                            const Eigen::Matrix4d& initial_guess = Eigen::Matrix4d::Identity());

    Eigen::Matrix4d currentPose() const { return current_pose_; }
    const VoxelHashMap& localMap() const { return local_map_; }
    std::vector<Eigen::Vector3d> getMapPoints() const { return local_map_.getAllPoints(); }

    void reset();

private:
    std::vector<Eigen::Vector3d> preprocessFrame(const std::vector<Eigen::Vector3d>& frame) const;
    std::vector<Eigen::Vector3d> voxelDownsample(const std::vector<Eigen::Vector3d>& points,
                                                  double voxel_size) const;

    ICPResult pointToPlaneICP(const std::vector<Eigen::Vector3d>& source,
                              const Eigen::Matrix4d& initial_guess);

    double computeAdaptiveThreshold();

    KissICPConfig config_;
    VoxelHashMap local_map_;
    Eigen::Matrix4d current_pose_ = Eigen::Matrix4d::Identity();

    // Adaptive threshold tracking
    std::vector<double> model_deviations_;
    bool has_moved_ = false;
};

}  // namespace g1_slam
