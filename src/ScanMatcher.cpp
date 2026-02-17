#include "ScanMatcher.hpp"
#include <iostream>
#include <cmath>

namespace g1_localization {

ScanMatcher::ScanMatcher() : max_iterations_(30), tolerance_(1e-4), outlier_dist_sq_(4.0f) {}

std::vector<LidarPoint> ScanMatcher::computeStructure(const std::vector<Eigen::Vector2f>& points) {
    std::vector<LidarPoint> structured;
    structured.reserve(points.size());

    // Simple normal estimation using neighbors
    for (size_t i = 0; i < points.size(); ++i) {
        LidarPoint lp(points[i].x(), points[i].y());
        
        // Find neighbors to fit a line
        Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
        int count = 0;
        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        
        for (const auto& p : points) {
            if ((p - points[i]).squaredNorm() < 0.15f) { // 15cm radius
                centroid += p;
                count++;
            }
        }
        
        if (count > 3) {
            centroid /= count;
            for (const auto& p : points) {
                if ((p - points[i]).squaredNorm() < 0.15f) {
                    Eigen::Vector2f d = p - centroid;
                    cov += d * d.transpose();
                }
            }
            // Eigen values of covariance -> smallest eigen vector is normal
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
            lp.normal = es.eigenvectors().col(0); 
            lp.valid_normal = true;
        }
        structured.push_back(lp);
    }
    return structured;
}

ScanMatcher::Result ScanMatcher::align(const std::vector<Eigen::Vector2f>& source,
                                       const std::vector<LidarPoint>& target,
                                       const Eigen::Vector2f& init_t,
                                       float init_theta) {
    Eigen::Vector2f t = init_t;
    float theta = init_theta;
    bool converged = false;
    float avg_error = 0.0f;
    int matches = 0;  // Moved outside loop for final check

    for (int iter = 0; iter < max_iterations_; ++iter) {
        float c = std::cos(theta), s = std::sin(theta);
        Eigen::Matrix2f R; R << c, -s, s, c;
        Eigen::Matrix2f R_deriv; R_deriv << -s, -c, c, -s; // dR/dTheta

        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f b = Eigen::Vector3f::Zero();
        float total_err = 0;
        matches = 0;  // Reset each iteration

        for (const auto& pt_src : source) {
            Eigen::Vector2f pt_curr = R * pt_src + t;

            // Find nearest in target (Brute force is OK for <1000 pts)
            int best_idx = -1;
            float min_dist = outlier_dist_sq_;

            for (size_t i = 0; i < target.size(); ++i) {
                float d = (pt_curr - target[i].pt).squaredNorm();
                if (d < min_dist) {
                    min_dist = d;
                    best_idx = i;
                }
            }

            if (best_idx != -1 && target[best_idx].valid_normal) {
                // Point-to-Line Error
                const auto& q = target[best_idx].pt;
                const auto& n = target[best_idx].normal;
                
                Eigen::Vector2f err_vec = pt_curr - q;
                float error = n.dot(err_vec);
                
                // Jacobian: J = [n_x, n_y, n^T * dR/dTheta * p]
                float jac_theta = n.dot(R_deriv * pt_src);
                Eigen::Vector3f J(n.x(), n.y(), jac_theta);

                // M-Estimator Weighting
                float weight = 1.0f / (1.0f + 10.0f * std::abs(error)); 

                H += weight * J * J.transpose();
                b -= weight * J * error;
                
                total_err += std::abs(error);
                matches++;
            }
        }

        // CLONE MISSING PIECE: The Prior (Kalman Filter behavior)
        // DIAGNOSTIC: Re-enabled.
        float pos_stiffness = 10.0f; 
        float rot_stiffness = 10.0f; // Very low penalty to allow large 'snaps' to walls
        
        Eigen::Matrix3f Prior_H = Eigen::Matrix3f::Identity();
        Prior_H(0,0) = pos_stiffness;
        Prior_H(1,1) = pos_stiffness;
        Prior_H(2,2) = rot_stiffness; // Trust IMU orientation heavily
        
        // Error from prediction (Current T - Initial T)
        Eigen::Vector3f dt_vec;
        dt_vec << (t.x() - init_t.x()), (t.y() - init_t.y()), (theta - init_theta);
        
        H += Prior_H;
        b -= Prior_H * dt_vec; // Pull back towards prediction

        if (matches < 10) break;
        avg_error = total_err / matches;

        // Relaxed convergence: Trust the error metric instead of gradient norm
        // The strict gradient threshold (1e-4) rarely succeeds in real environments
        if (iter > 0 && b.norm() < 1e-3) {  // Relaxed from 1e-4 to 1e-3
            converged = true;
            break;
        }
        
        // Also mark as converged if error is stable and low
        if (iter >= 5 && avg_error < 0.20f) {
            converged = true;
        }

        Eigen::Vector3f delta = H.ldlt().solve(b);
        t.x() += delta(0);
        t.y() += delta(1);
        theta += delta(2);
    }
    
    // Final convergence check: If we have good matches and low error, accept it
    if (matches >= 20 && avg_error < 0.25f) {
        converged = true;
    }
    
    return {Eigen::Rotation2D<float>(theta).toRotationMatrix(), t, avg_error, converged};
}

} // namespace g1_localization
