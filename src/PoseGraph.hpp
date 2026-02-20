#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cstdint>

namespace g1_localization {

struct Pose2 {
    float x = 0.f;
    float y = 0.f;
    float theta = 0.f;
};

struct Keyframe {
    int id = -1;
    Pose2 pose_odom;   // odometry pose when keyframe was created
    Pose2 pose_opt;    // optimized pose (same as odom until optimization runs)
    std::vector<Eigen::Vector2f> scan;  // scan in body frame at keyframe
};

struct PoseGraphEdge {
    int id_from = -1;
    int id_to = -1;
    float dx = 0.f, dy = 0.f, dtheta = 0.f;  // measurement: pose_to = from + (dx,dy,dtheta) in from's frame
    float weight_pos = 1.f;
    float weight_theta = 1.f;
    bool is_loop = false;
};

class ScanMatcher;

class PoseGraph {
public:
    PoseGraph();

    // Add keyframe; returns keyframe id. Adds odometry edge from previous keyframe.
    int addKeyframe(float odom_x, float odom_y, float odom_theta,
                    const std::vector<Eigen::Vector2f>& scan);

    // Try loop closures for the latest keyframe against older keyframes; uses ICP.
    void tryLoopClosures(ScanMatcher* matcher, int max_candidates = 15,
                         float max_icp_error = 0.18f);

    // Run Gauss-Newton optimization over all node poses.
    void optimize(int max_iterations = 20);

    // Get optimized pose for a keyframe (by index in keyframes, 0 = oldest).
    Pose2 getOptimizedPose(int keyframe_index) const;

    // Get current pose in world (odom frame corrected by graph). Pass current odometry.
    void getCurrentPoseInWorld(float odom_x, float odom_y, float odom_theta,
                               float* out_x, float* out_y, float* out_theta) const;

    // Build global map from all keyframe scans transformed by optimized poses (for ICP).
    std::vector<Eigen::Vector2f> getMapPoints() const;

    // Number of keyframes
    size_t numKeyframes() const { return keyframes_.size(); }

    // Last keyframe id (for interpolation)
    int lastKeyframeId() const { return keyframes_.empty() ? -1 : keyframes_.back().id; }

    void setKeyframeDistanceThreshold(float dist_m, float angle_rad);
    bool shouldAddKeyframe(float odom_x, float odom_y, float odom_theta) const;

private:
    std::vector<Keyframe> keyframes_;
    std::vector<PoseGraphEdge> edges_;
    int next_id_ = 0;

    float keyframe_dist_thresh_ = 0.35f;
    float keyframe_angle_thresh_ = 0.25f;
    float last_kf_odom_x_ = 0.f, last_kf_odom_y_ = 0.f, last_kf_odom_theta_ = 0.f;
    bool has_last_kf_ = false;

    static constexpr int max_keyframes_ = 250;
};

} // namespace g1_localization
