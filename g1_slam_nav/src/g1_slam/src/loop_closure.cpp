#include "g1_slam/loop_closure.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>

namespace g1_slam {

LoopClosureDetector::LoopClosureDetector()
    : LoopClosureDetector(Config()) {}

LoopClosureDetector::LoopClosureDetector(const Config& config)
    : config_(config) {}

Eigen::VectorXd LoopClosureDetector::computeScanContext(
    const std::vector<Eigen::Vector3d>& points) const {

    int total = config_.descriptor_rings * config_.descriptor_sectors;
    Eigen::VectorXd desc = Eigen::VectorXd::Zero(total);

    for (const auto& p : points) {
        double range = std::sqrt(p.x() * p.x() + p.y() * p.y());
        if (range > config_.descriptor_max_range || range < 0.5) continue;

        double angle = std::atan2(p.y(), p.x()) + M_PI;  // [0, 2*PI]
        int ring = static_cast<int>(range / config_.descriptor_max_range * config_.descriptor_rings);
        int sector = static_cast<int>(angle / (2.0 * M_PI) * config_.descriptor_sectors);

        ring = std::clamp(ring, 0, config_.descriptor_rings - 1);
        sector = std::clamp(sector, 0, config_.descriptor_sectors - 1);

        int idx = ring * config_.descriptor_sectors + sector;
        desc(idx) = std::max(desc(idx), p.z());  // Max height in bin
    }

    return desc;
}

double LoopClosureDetector::computeDescriptorDistance(
    const Eigen::VectorXd& a, const Eigen::VectorXd& b) const {

    // Cosine distance with column shift for rotation invariance
    double best_score = 0.0;
    int sectors = config_.descriptor_sectors;
    int rings = config_.descriptor_rings;

    for (int shift = 0; shift < sectors; ++shift) {
        double score = 0.0;
        for (int r = 0; r < rings; ++r) {
            Eigen::VectorXd col_a = a.segment(r * sectors, sectors);
            Eigen::VectorXd col_b(sectors);
            for (int s = 0; s < sectors; ++s) {
                col_b(s) = b(r * sectors + (s + shift) % sectors);
            }
            double na = col_a.norm();
            double nb = col_b.norm();
            if (na > 1e-6 && nb > 1e-6) {
                score += col_a.dot(col_b) / (na * nb);
            }
        }
        best_score = std::max(best_score, score / rings);
    }

    return 1.0 - best_score;
}

bool LoopClosureDetector::addKeyframe(const Eigen::Matrix4d& pose,
                                       const std::vector<Eigen::Vector3d>& scan,
                                       double timestamp) {
    if (!keyframes_.empty()) {
        const auto& last = keyframes_.back();
        Eigen::Vector3d delta = pose.block<3,1>(0,3) - last.pose.block<3,1>(0,3);
        double dist = delta.norm();

        Eigen::Matrix3d R_delta = last.pose.block<3,3>(0,0).transpose() * pose.block<3,3>(0,0);
        double angle = std::acos(std::clamp((R_delta.trace() - 1.0) / 2.0, -1.0, 1.0));

        if (dist < config_.keyframe_distance && angle < config_.keyframe_angle) {
            return false;
        }
    }

    ScanContext kf;
    kf.id = next_id_++;
    kf.pose = pose;
    kf.keypoints = scan;  // Store downsampled scan
    kf.descriptor = computeScanContext(scan);
    kf.timestamp = timestamp;
    keyframes_.push_back(kf);
    return true;
}

std::optional<LoopConstraint> LoopClosureDetector::detectLoop() {
    if (keyframes_.size() < static_cast<size_t>(config_.min_keyframe_gap + 1)) {
        return std::nullopt;
    }

    const auto& query = keyframes_.back();
    double best_score = std::numeric_limits<double>::max();
    int best_idx = -1;

    for (size_t i = 0; i < keyframes_.size() - config_.min_keyframe_gap; ++i) {
        const auto& candidate = keyframes_[i];

        // Distance check
        double dist = (query.pose.block<3,1>(0,3) - candidate.pose.block<3,1>(0,3)).norm();
        if (dist > config_.search_radius) continue;

        double desc_dist = computeDescriptorDistance(query.descriptor, candidate.descriptor);
        if (desc_dist < best_score) {
            best_score = desc_dist;
            best_idx = static_cast<int>(i);
        }
    }

    if (best_idx < 0 || best_score > 0.4) {
        return std::nullopt;
    }

    return verifyLoop(query, keyframes_[best_idx]);
}

std::optional<LoopConstraint> LoopClosureDetector::verifyLoop(
    const ScanContext& query, const ScanContext& candidate) const {

    // ICP verification between the two keyframe scans
    // Transform candidate scan to query frame using relative pose guess
    Eigen::Matrix4d T_rel = candidate.pose.inverse() * query.pose;
    Eigen::Matrix3d R_rel = T_rel.block<3,3>(0,0);
    Eigen::Vector3d t_rel = T_rel.block<3,1>(0,3);

    // Simple point-to-point ICP for verification
    std::vector<Eigen::Vector3d> src = query.keypoints;
    const auto& tgt = candidate.keypoints;

    if (src.size() < 50 || tgt.size() < 50) return std::nullopt;

    Eigen::Matrix3d R = R_rel;
    Eigen::Vector3d t = t_rel;

    for (int iter = 0; iter < 30; ++iter) {
        double total_err = 0.0;
        int count = 0;
        Eigen::Vector3d mean_src = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_tgt = Eigen::Vector3d::Zero();

        std::vector<std::pair<int, int>> correspondences;
        for (size_t i = 0; i < src.size(); i += 3) {
            Eigen::Vector3d sp = R * src[i] + t;
            double best_d = std::numeric_limits<double>::max();
            int best_j = -1;
            for (size_t j = 0; j < tgt.size(); j += 3) {
                double d = (sp - tgt[j]).squaredNorm();
                if (d < best_d) { best_d = d; best_j = j; }
            }
            if (best_d < 4.0) {
                correspondences.push_back({static_cast<int>(i), best_j});
                mean_src += src[i];
                mean_tgt += tgt[best_j];
                total_err += best_d;
                count++;
            }
        }

        if (count < 20) return std::nullopt;

        mean_src /= count;
        mean_tgt /= count;

        Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
        for (const auto& [si, ti] : correspondences) {
            W += (src[si] - mean_src) * (tgt[ti] - mean_tgt).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixV() * svd.matrixU().transpose();
        if (R.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1;
            R = V * svd.matrixU().transpose();
        }
        t = mean_tgt - R * mean_src;

        double fitness = total_err / count;
        if (fitness < config_.fitness_threshold * config_.fitness_threshold) {
            LoopConstraint lc;
            lc.from_id = candidate.id;
            lc.to_id = query.id;
            Eigen::Matrix4d T_loop = Eigen::Matrix4d::Identity();
            T_loop.block<3,3>(0,0) = R;
            T_loop.block<3,1>(0,3) = t;
            lc.relative_pose = T_loop;
            lc.information = Eigen::Matrix<double, 6, 6>::Identity() * (1.0 / std::max(fitness, 0.001));
            lc.score = std::sqrt(fitness);
            return lc;
        }
    }

    return std::nullopt;
}

}  // namespace g1_slam
