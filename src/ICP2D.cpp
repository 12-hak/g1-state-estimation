#include "ICP2D.hpp"
#include <iostream>
#include <numeric>

namespace g1_localization {

// Constructor matching header declaration
ICP2D::ICP2D(int max_iterations, float tolerance, float outlier_threshold)
    : max_iterations_(max_iterations), tolerance_(tolerance), outlier_threshold_(outlier_threshold) {}

ICPResult ICP2D::align(const std::vector<Eigen::Vector2f>& source,
                       const std::vector<Eigen::Vector2f>& target) {
    // Return with 5 fields matching ICPResult struct in hpp
    if (source.empty() || target.empty()) {
        return {Eigen::Matrix2f::Identity(), Eigen::Vector2f::Zero(), false, 0.0f, 0.0f};
    }

    Eigen::Matrix2f R = Eigen::Matrix2f::Identity();
    Eigen::Vector2f t = Eigen::Vector2f::Zero();
    float total_error = 0.0f;
    bool converged = false;
    float inlier_ratio = 0.0f;

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
                // Fixed Eigen syntax error
                float d_sq = (s_trans - target[i]).squaredNorm();
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

        if (src_matched.size() < 10) break; // Not enough points

        // Compute centroids
        Eigen::Vector2f src_mean = Eigen::Vector2f::Zero();
        Eigen::Vector2f tgt_mean = Eigen::Vector2f::Zero();
        for(const auto& p : src_matched) src_mean += p;
        for(const auto& p : tgt_matched) tgt_mean += p;
        src_mean /= src_matched.size();
        tgt_mean /= tgt_matched.size();

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

        // Apply new transformation
        // Note: This logic seems to calculate absolute R from source to target?
        // Standard ICP iteratively updates R and t.
        // Let's assume standard SVD logic:
        // T_new aligns (src - src_mean) to (tgt - tgt_mean).
        // pt_tgt = R_new * (pt_src - src_mean) + tgt_mean
        // pt_tgt = R_new * pt_src + (tgt_mean - R_new * src_mean)
        
        // Accumulate transformation? No, usually we solve for absolute R,t if we re-transform source.
        // But here loop line 25 uses R*s + t.
        // So R and t are cumulative.
        // SVD gives best R,t between src_matched (ORIGINAL points) and tgt_matched?
        // Wait, src_matched stores 's' (Original Points). Correct.
        // tgt_matched stores nearest point in TARGET.
        // So SVD solves alignment between Original Source and Current Target Estimate.
        // So R and t are indeed Absolute.
        
        Eigen::Vector2f t_new = tgt_mean - R_new * src_mean;

        // Check convergence
        float delta_t = (t_new - t).norm();
        R = R_new;
        t = t_new;
        total_error = current_error / src_matched.size();
        inlier_ratio = (float)src_matched.size() / source.size();

        if (delta_t < tolerance_) {
            converged = true;
            break;
        }
    }

    return {R, t, converged, total_error, inlier_ratio};
}

} // namespace g1_localization
