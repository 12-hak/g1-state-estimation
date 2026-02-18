#include "g1_slam/kiss_icp_wrapper.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

namespace g1_slam {

static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<    0, -v.z(),  v.y(),
         v.z(),     0, -v.x(),
        -v.y(),  v.x(),     0;
    return S;
}

KissICPWrapper::KissICPWrapper(const KissICPConfig& config)
    : config_(config), local_map_(config.voxel_size) {}

void KissICPWrapper::reset() {
    current_pose_ = Eigen::Matrix4d::Identity();
    local_map_.clear();
    model_deviations_.clear();
    has_moved_ = false;
}

std::vector<Eigen::Vector3d> KissICPWrapper::preprocessFrame(
    const std::vector<Eigen::Vector3d>& frame) const {

    std::vector<Eigen::Vector3d> filtered;
    filtered.reserve(frame.size());

    double min_sq = config_.min_range * config_.min_range;
    double max_sq = config_.max_range * config_.max_range;

    for (const auto& p : frame) {
        double dist_sq = p.squaredNorm();
        if (dist_sq >= min_sq && dist_sq <= max_sq) {
            filtered.push_back(p);
        }
    }

    return voxelDownsample(filtered, config_.voxel_size * 0.5);
}

std::vector<Eigen::Vector3d> KissICPWrapper::voxelDownsample(
    const std::vector<Eigen::Vector3d>& points, double voxel_size) const {

    struct VKey {
        int32_t x, y, z;
        bool operator==(const VKey& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct VKeyHash {
        size_t operator()(const VKey& k) const {
            return 73856093ULL * k.x ^ 19349669ULL * k.y ^ 83492791ULL * k.z;
        }
    };

    std::unordered_map<VKey, Eigen::Vector3d, VKeyHash> grid;
    std::unordered_map<VKey, int, VKeyHash> counts;

    for (const auto& p : points) {
        VKey key{
            static_cast<int32_t>(std::floor(p.x() / voxel_size)),
            static_cast<int32_t>(std::floor(p.y() / voxel_size)),
            static_cast<int32_t>(std::floor(p.z() / voxel_size))
        };
        auto it = grid.find(key);
        if (it == grid.end()) {
            grid[key] = p;
            counts[key] = 1;
        } else {
            int n = counts[key]++;
            it->second = (it->second * n + p) / (n + 1);
        }
    }

    std::vector<Eigen::Vector3d> result;
    result.reserve(grid.size());
    for (const auto& [key, centroid] : grid) {
        result.push_back(centroid);
    }
    return result;
}

ICPResult KissICPWrapper::pointToPlaneICP(
    const std::vector<Eigen::Vector3d>& source,
    const Eigen::Matrix4d& initial_guess) {

    ICPResult result;
    result.pose = initial_guess;

    if (local_map_.size() == 0) {
        result.converged = true;
        return result;
    }

    Eigen::Matrix4d T = initial_guess;
    double adaptive_threshold = computeAdaptiveThreshold();
    double threshold_sq = adaptive_threshold * adaptive_threshold;

    for (int iter = 0; iter < config_.max_iterations; ++iter) {
        Eigen::Matrix3d R = T.block<3,3>(0,0);
        Eigen::Vector3d t = T.block<3,1>(0,3);

        // Transform source points
        std::vector<Eigen::Vector3d> src_transformed(source.size());
        for (size_t i = 0; i < source.size(); ++i) {
            src_transformed[i] = R * source[i] + t;
        }

        // Find correspondences and compute point-to-point residuals
        // (point-to-plane with estimated normals from local neighborhood)
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
        int num_correspondences = 0;
        double total_error = 0.0;

        for (const auto& sp : src_transformed) {
            auto neighbors = local_map_.getPointsNear(sp, adaptive_threshold);
            if (neighbors.empty()) continue;

            // Find closest point
            double best_dist_sq = std::numeric_limits<double>::max();
            Eigen::Vector3d closest;
            for (const auto& np : neighbors) {
                double d = (sp - np).squaredNorm();
                if (d < best_dist_sq) {
                    best_dist_sq = d;
                    closest = np;
                }
            }

            if (best_dist_sq > threshold_sq) continue;

            // Estimate normal from local neighborhood via PCA
            Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
            if (neighbors.size() >= 3) {
                Eigen::Vector3d mean = Eigen::Vector3d::Zero();
                for (const auto& n : neighbors) mean += n;
                mean /= static_cast<double>(neighbors.size());

                Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                for (const auto& n : neighbors) {
                    Eigen::Vector3d d = n - mean;
                    cov += d * d.transpose();
                }
                cov /= static_cast<double>(neighbors.size());

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
                normal = solver.eigenvectors().col(0);  // Smallest eigenvalue = normal
            }

            Eigen::Vector3d residual_vec = sp - closest;
            double residual = residual_vec.dot(normal);
            total_error += residual * residual;

            // Jacobian: d(residual)/d(xi) where xi = [tx, ty, tz, rx, ry, rz]
            Eigen::Matrix<double, 1, 6> J;
            J.block<1,3>(0,0) = normal.transpose();
            J.block<1,3>(0,3) = -normal.transpose() * skew(sp);

            H += J.transpose() * J;
            b += J.transpose() * (-residual);
            num_correspondences++;
        }

        if (num_correspondences < 10) break;

        // Solve H * dx = b
        Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(b);

        // Update transform
        Eigen::Matrix4d dT = Eigen::Matrix4d::Identity();
        dT.block<3,3>(0,0) = (Eigen::AngleAxisd(dx(5), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(dx(4), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(dx(3), Eigen::Vector3d::UnitX())).toRotationMatrix();
        dT.block<3,1>(0,3) = dx.head<3>();

        T = dT * T;
        result.iterations = iter + 1;
        result.fitness_score = total_error / num_correspondences;

        if (dx.norm() < config_.convergence_threshold) {
            result.converged = true;
            break;
        }
    }

    result.pose = T;
    if (result.iterations == config_.max_iterations) {
        result.converged = (result.fitness_score < 0.5);
    }

    return result;
}

ICPResult KissICPWrapper::registerFrame(
    const std::vector<Eigen::Vector3d>& frame,
    const Eigen::Matrix4d& initial_guess) {

    auto preprocessed = preprocessFrame(frame);
    if (preprocessed.empty()) {
        ICPResult r;
        r.pose = current_pose_;
        return r;
    }

    Eigen::Matrix4d guess = initial_guess;
    if (guess.isIdentity()) {
        guess = current_pose_;
    }

    auto result = pointToPlaneICP(preprocessed, guess);

    if (result.converged) {
        // Track motion for adaptive threshold
        Eigen::Vector3d delta = result.pose.block<3,1>(0,3) - current_pose_.block<3,1>(0,3);
        model_deviations_.push_back(delta.norm());
        if (model_deviations_.size() > 100) {
            model_deviations_.erase(model_deviations_.begin());
        }

        current_pose_ = result.pose;

        if (!has_moved_ && delta.norm() > config_.min_motion_threshold) {
            has_moved_ = true;
        }

        // Add transformed points to local map
        Eigen::Matrix3d R = current_pose_.block<3,3>(0,0);
        Eigen::Vector3d t = current_pose_.block<3,1>(0,3);
        std::vector<Eigen::Vector3d> world_points(preprocessed.size());
        for (size_t i = 0; i < preprocessed.size(); ++i) {
            world_points[i] = R * preprocessed[i] + t;
        }
        local_map_.addPoints(world_points);

        // Trim far-away points to bound map size
        local_map_.removePointsFarFrom(t, config_.max_range * 3.0);
    }

    return result;
}

double KissICPWrapper::computeAdaptiveThreshold() {
    if (model_deviations_.empty()) {
        return config_.initial_threshold;
    }
    auto sorted = model_deviations_;
    std::sort(sorted.begin(), sorted.end());
    double median = sorted[sorted.size() / 2];
    return std::max(config_.initial_threshold * 0.1, median * 3.0);
}

}  // namespace g1_slam
