#include "ICP2D.hpp"
#include <iostream>
#include <numeric>

namespace g1_localization {

ICP2D::ICP2D() : max_iterations_(30), tolerance_(1e-4), outlier_threshold_(0.5f) {}

ICP2D::Result ICP2D::align(const std::vector<Eigen::Vector2f>& source,
                            const std::vector<Eigen::Vector2f>& target) {
    if (source.empty() || target.empty()) return {Eigen::Matrix2f::Identity(), Eigen::Vector2f::Zero(), 0.0f, false};

    Eigen::Matrix2f R = Eigen::Matrix2f::Identity();
    Eigen::Vector2f t = Eigen::Vector2f::Zero();
    float total_error = 0.0f;
    bool converged = false;

    // We align the SOURCE (current scan) to the TARGET (the fixed map)
    for (int iter = 0; iter < max_iterations_; ++iter) {
        std::vector<Eigen::Vector2f> src_matched;
        std::vector<Eigen::Vector2f> tgt_matched;
        float current_error = 0.0f;

        for (const auto& s : source) {
            Eigen::Vector2f s_trans = R * s + t;
            
            // Find nearest neighbor in target
            float min_dist_sq = outlier_threshold_ * outlier_threshold_;
            int best_idx = -1;
            
            for (size_t i = 0; i < target.size(); ++i) {
                float d_sq = (s_trans - target[i]).get_shape().is_empty() ? 0 : (s_trans - target[i]).squaredNorm();
                if (d_sq < min_dist_sq) {
                    min_dist_sq = d_sq;
                    best_idx = i;
                }
            }

            if (best_idx != -1) {
                src_matched.push_back(s);
                tgt_matched.push_back(target[best_idx]);
                current_error += std::sqrt(min_dist_sq);
            }
        }

        if (src_matched.size() < 10) break;

        // Compute centroids
        Eigen::Vector2f src_mean = std::accumulate(src_matched.begin(), src_matched.end(), Eigen::Vector2f::Zero().eval()) / src_matched.size();
        Eigen::Vector2f tgt_mean = std::accumulate(tgt_matched.begin(), tgt_matched.end(), Eigen::Vector2f::Zero().eval()) / tgt_matched.size();

        // SVD-based rotation matching
        Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
        for (size_t i = 0; i < src_matched.size(); ++i) {
            H += (src_matched[i] - src_mean) * (tgt_matched[i] - tgt_mean).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2f R_new = svd.matrixV() * svd.matrixU().transpose();

        if (R_new.determinant() < 0) {
            Eigen::Matrix2f V = svd.matrixV();
            V.col(1) *= -1;
            R_new = V * svd.matrixU().transpose();
        }

        Eigen::Vector2f t_new = tgt_mean - R_new * src_mean;

        // Check convergence
        float delta_t = (t_new - t).norm();
        R = R_new;
        t = t_new;
        total_error = current_error / src_matched.size();

        if (delta_t < tolerance_) {
            converged = true;
            break;
        }
    }

    return {R, t, total_error, converged};
}

} // namespace g1_localization
