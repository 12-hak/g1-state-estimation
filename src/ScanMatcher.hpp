#pragma once

#include <Eigen/Dense>
#include <vector>

namespace g1_localization {

struct LidarPoint {
    Eigen::Vector2f pt;
    Eigen::Vector2f normal; // For Point-to-Plane
    bool valid_normal = false;
    
    LidarPoint(float x, float y) : pt(x, y), valid_normal(false) {}
};

class ScanMatcher {
public:
    struct Result {
        Eigen::Matrix2f rotation;
        Eigen::Vector2f translation;
        float error;
        bool converged;
    };

    ScanMatcher();

    // The heavy lifter: Point-to-Line ICP
    Result align(const std::vector<Eigen::Vector2f>& source,
                 const std::vector<LidarPoint>& target_map,
                 const Eigen::Vector2f& initial_guess_t,
                 float initial_guess_angle);

    // Helper to compute map normals (features)
    static std::vector<LidarPoint> computeStructure(const std::vector<Eigen::Vector2f>& points);

private:
    int max_iterations_;
    float tolerance_;
    float outlier_dist_sq_;
};

} // namespace g1_localization
