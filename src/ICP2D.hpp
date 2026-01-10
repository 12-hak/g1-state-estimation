#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace g1_localization {

struct ICPResult {
    Eigen::Matrix2f rotation;
    Eigen::Vector2f translation;
    bool converged;
    float error;
    float inlier_ratio;
};

class ICP2D {
public:
    ICP2D(int max_iterations = 30, float tolerance = 1e-5f, float outlier_threshold = 3.0f);
    
    ICPResult align(const std::vector<Eigen::Vector2f>& source,
                    const std::vector<Eigen::Vector2f>& target);

private:
    int max_iterations_;
    float tolerance_;
    float outlier_threshold_;
    
    std::vector<int> findNearestNeighbors(const std::vector<Eigen::Vector2f>& source,
                                          const std::vector<Eigen::Vector2f>& target,
                                          std::vector<float>& distances);
    
    std::vector<bool> rejectOutliers(const std::vector<float>& distances);
};

} // namespace g1_localization
