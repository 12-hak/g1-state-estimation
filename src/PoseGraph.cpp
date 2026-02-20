#include "PoseGraph.hpp"
#include "ScanMatcher.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

namespace g1_localization {

static constexpr float PI = 3.14159265358979323846f;

static float normalizeAngle(float a) {
    while (a > PI) a -= 2.f * PI;
    while (a < -PI) a += 2.f * PI;
    return a;
}

PoseGraph::PoseGraph() = default;

void PoseGraph::setKeyframeDistanceThreshold(float dist_m, float angle_rad) {
    keyframe_dist_thresh_ = dist_m;
    keyframe_angle_thresh_ = angle_rad;
}

bool PoseGraph::shouldAddKeyframe(float odom_x, float odom_y, float odom_theta) const {
    if (!has_last_kf_) return true;
    float dx = odom_x - last_kf_odom_x_;
    float dy = odom_y - last_kf_odom_y_;
    float dtheta = std::abs(normalizeAngle(odom_theta - last_kf_odom_theta_));
    float dist = std::sqrt(dx*dx + dy*dy);
    return (dist >= keyframe_dist_thresh_ || dtheta >= keyframe_angle_thresh_);
}

int PoseGraph::addKeyframe(float odom_x, float odom_y, float odom_theta,
                           const std::vector<Eigen::Vector2f>& scan) {
    Keyframe kf;
    kf.id = next_id_++;
    kf.pose_odom.x = odom_x;
    kf.pose_odom.y = odom_y;
    kf.pose_odom.theta = odom_theta;
    kf.pose_opt = kf.pose_odom;
    kf.scan = scan;

    if (!keyframes_.empty()) {
        const auto& prev = keyframes_.back();
        PoseGraphEdge e;
        e.id_from = prev.id;
        e.id_to = kf.id;
        float c = std::cos(-prev.pose_odom.theta), s = std::sin(-prev.pose_odom.theta);
        e.dx = c * (odom_x - prev.pose_odom.x) - s * (odom_y - prev.pose_odom.y);
        e.dy = s * (odom_x - prev.pose_odom.x) + c * (odom_y - prev.pose_odom.y);
        e.dtheta = normalizeAngle(odom_theta - prev.pose_odom.theta);
        e.is_loop = false;
        e.weight_pos = 1.f;
        e.weight_theta = 1.f;
        edges_.push_back(e);
    }

    keyframes_.push_back(std::move(kf));
    last_kf_odom_x_ = odom_x;
    last_kf_odom_y_ = odom_y;
    last_kf_odom_theta_ = odom_theta;
    has_last_kf_ = true;

    if (static_cast<int>(keyframes_.size()) > max_keyframes_) {
        int removed_id = keyframes_.front().id;
        keyframes_.erase(keyframes_.begin());
        edges_.erase(std::remove_if(edges_.begin(), edges_.end(),
            [removed_id](const PoseGraphEdge& e) {
                return e.id_from == removed_id || e.id_to == removed_id;
            }), edges_.end());
    }
    return keyframes_.back().id;
}

void PoseGraph::tryLoopClosures(ScanMatcher* matcher, int max_candidates, float max_icp_error) {
    if (!matcher || keyframes_.size() < 2) return;
    const int last_idx = static_cast<int>(keyframes_.size()) - 1;
    const Keyframe& last_kf = keyframes_[last_idx];
    std::vector<LidarPoint> last_structure = ScanMatcher::computeStructure(last_kf.scan);

    int tried = 0;
    for (int i = 0; i < last_idx && tried < max_candidates; ++i) {
        if (last_idx - i < 5) continue;
        tried++;
        const Keyframe& old_kf = keyframes_[i];
        float dx = last_kf.pose_odom.x - old_kf.pose_odom.x;
        float dy = last_kf.pose_odom.y - old_kf.pose_odom.y;
        if (dx*dx + dy*dy > 100.f) continue;

        std::vector<Eigen::Vector2f> old_world;
        float co = std::cos(old_kf.pose_opt.theta), so = std::sin(old_kf.pose_opt.theta);
        for (const auto& p : old_kf.scan) {
            old_world.emplace_back(co * p.x() - so * p.y() + old_kf.pose_opt.x,
                                  so * p.x() + co * p.y() + old_kf.pose_opt.y);
        }
        auto target_map = ScanMatcher::computeStructure(old_world);
        Eigen::Vector2f init_t(last_kf.pose_opt.x, last_kf.pose_opt.y);
        float init_theta = last_kf.pose_opt.theta;

        auto result = matcher->align(last_kf.scan, target_map, init_t, init_theta);
        if (!result.converged || result.error >= max_icp_error) continue;

        float icp_yaw = std::atan2(result.rotation(1,0), result.rotation(0,0));
        float rel_x = result.translation.x() - old_kf.pose_opt.x;
        float rel_y = result.translation.y() - old_kf.pose_opt.y;
        float c = std::cos(-old_kf.pose_opt.theta), s = std::sin(-old_kf.pose_opt.theta);
        PoseGraphEdge e;
        e.id_from = old_kf.id;
        e.id_to = last_kf.id;
        e.dx = c * rel_x - s * rel_y;
        e.dy = s * rel_x + c * rel_y;
        e.dtheta = normalizeAngle(icp_yaw - old_kf.pose_opt.theta);
        e.is_loop = true;
        e.weight_pos = 2.f;
        e.weight_theta = 2.f;
        edges_.push_back(e);
        std::cout << "[PoseGraph] Loop closure: kf " << old_kf.id << " -> " << last_kf.id << ", error=" << result.error << std::endl;
    }
}

void PoseGraph::optimize(int max_iterations) {
    if (keyframes_.size() < 2) return;
    const int n = static_cast<int>(keyframes_.size());
    for (int iter = 0; iter < max_iterations; ++iter) {
        Eigen::VectorXf b = Eigen::VectorXf::Zero(3 * n);
        Eigen::MatrixXf H = Eigen::MatrixXf::Zero(3 * n, 3 * n);
        for (const auto& e : edges_) {
            int i_from = -1, i_to = -1;
            for (int k = 0; k < n; ++k) {
                if (keyframes_[k].id == e.id_from) i_from = k;
                if (keyframes_[k].id == e.id_to) i_to = k;
            }
            if (i_from < 0 || i_to < 0) continue;
            const auto& from = keyframes_[i_from].pose_opt;
            const auto& to = keyframes_[i_to].pose_opt;
            float c = std::cos(from.theta), s = std::sin(from.theta);
            float pred_x = from.x + c * e.dx - s * e.dy;
            float pred_y = from.y + s * e.dx + c * e.dy;
            float pred_theta = normalizeAngle(from.theta + e.dtheta);
            float rx = to.x - pred_x;
            float ry = to.y - pred_y;
            float rtheta = normalizeAngle(to.theta - pred_theta);
            float wp = e.weight_pos, wt = e.weight_theta;
            Eigen::Vector3f r(rx, ry, rtheta);
            Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
            W(0,0) = W(1,1) = wp; W(2,2) = wt;
            Eigen::Matrix3f J_to = Eigen::Matrix3f::Identity();
            Eigen::Matrix3f J_from = Eigen::Matrix3f::Zero();
            J_from(0,0) = -1; J_from(0,2) = s * e.dx + c * e.dy;
            J_from(1,1) = -1; J_from(1,2) = -c * e.dx + s * e.dy;
            J_from(2,2) = -1;
            Eigen::Matrix3f JtW = J_to.transpose() * W;
            b.segment<3>(3*i_to) -= JtW * r;
            b.segment<3>(3*i_from) -= J_from.transpose() * W * r;
            H.block<3,3>(3*i_to, 3*i_to) += JtW * J_to;
            H.block<3,3>(3*i_from, 3*i_from) += J_from.transpose() * W * J_from;
            Eigen::Matrix3f cross = JtW * J_from;
            H.block<3,3>(3*i_to, 3*i_from) += cross;
            H.block<3,3>(3*i_from, 3*i_to) += cross.transpose();
        }
        H.diagonal().array() += 1e-5f;
        Eigen::VectorXf dx = H.ldlt().solve(b);
        for (int k = 0; k < n; ++k) {
            keyframes_[k].pose_opt.x += dx(3*k);
            keyframes_[k].pose_opt.y += dx(3*k+1);
            keyframes_[k].pose_opt.theta = normalizeAngle(keyframes_[k].pose_opt.theta + dx(3*k+2));
        }
        if (dx.norm() < 1e-4f) break;
    }
}

Pose2 PoseGraph::getOptimizedPose(int keyframe_index) const {
    if (keyframe_index < 0 || keyframe_index >= static_cast<int>(keyframes_.size()))
        return Pose2{0.f, 0.f, 0.f};
    return keyframes_[keyframe_index].pose_opt;
}

void PoseGraph::getCurrentPoseInWorld(float odom_x, float odom_y, float odom_theta,
                                      float* out_x, float* out_y, float* out_theta) const {
    if (keyframes_.empty()) {
        *out_x = odom_x; *out_y = odom_y; *out_theta = odom_theta;
        return;
    }
    const auto& last = keyframes_.back();
    float dx = odom_x - last.pose_odom.x;
    float dy = odom_y - last.pose_odom.y;
    float dtheta = normalizeAngle(odom_theta - last.pose_odom.theta);
    float c = std::cos(last.pose_opt.theta), s = std::sin(last.pose_opt.theta);
    *out_x = last.pose_opt.x + c * dx - s * dy;
    *out_y = last.pose_opt.y + s * dx + c * dy;
    *out_theta = normalizeAngle(last.pose_opt.theta + dtheta);
}

std::vector<Eigen::Vector2f> PoseGraph::getMapPoints() const {
    std::vector<Eigen::Vector2f> out;
    for (const auto& kf : keyframes_) {
        float c = std::cos(kf.pose_opt.theta), s = std::sin(kf.pose_opt.theta);
        for (const auto& p : kf.scan) {
            out.emplace_back(c * p.x() - s * p.y() + kf.pose_opt.x,
                             s * p.x() + c * p.y() + kf.pose_opt.y);
        }
    }
    return out;
}

} // namespace g1_localization
