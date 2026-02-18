#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <optional>

namespace g1_slam {

struct ScanContext {
    int id;
    Eigen::Matrix4d pose;
    std::vector<Eigen::Vector3d> keypoints;
    Eigen::VectorXd descriptor;
    double timestamp;
};

struct LoopConstraint {
    int from_id;
    int to_id;
    Eigen::Matrix4d relative_pose;
    Eigen::Matrix<double, 6, 6> information;
    double score;
};

class LoopClosureDetector {
public:
    struct Config {
        double keyframe_distance = 1.0;        // Minimum distance between keyframes (m)
        double keyframe_angle = 0.5;            // Minimum angle between keyframes (rad)
        double search_radius = 15.0;            // Loop search radius (m)
        double min_loop_distance = 5.0;         // Minimum travel distance before accepting loop
        int min_keyframe_gap = 20;              // Minimum keyframe gap for loop candidates
        double fitness_threshold = 0.3;         // ICP fitness score threshold
        int descriptor_rings = 20;              // Scan context rings
        int descriptor_sectors = 60;            // Scan context sectors
        double descriptor_max_range = 15.0;     // Max range for scan context
    };

    explicit LoopClosureDetector(const Config& config = Config());

    bool addKeyframe(const Eigen::Matrix4d& pose,
                     const std::vector<Eigen::Vector3d>& scan,
                     double timestamp);

    std::optional<LoopConstraint> detectLoop();

    const std::vector<ScanContext>& keyframes() const { return keyframes_; }
    size_t numKeyframes() const { return keyframes_.size(); }

private:
    Eigen::VectorXd computeScanContext(const std::vector<Eigen::Vector3d>& points) const;
    double computeDescriptorDistance(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const;

    std::optional<LoopConstraint> verifyLoop(const ScanContext& query,
                                              const ScanContext& candidate) const;

    Config config_;
    std::vector<ScanContext> keyframes_;
    int next_id_ = 0;
};

}  // namespace g1_slam
