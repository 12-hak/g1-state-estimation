#include "g1_slam/voxel_hash_map.hpp"
#include <cmath>
#include <algorithm>

namespace g1_slam {

VoxelHashMap::VoxelHashMap(double voxel_size)
    : voxel_size_(voxel_size) {}

VoxelKey VoxelHashMap::toKey(const Eigen::Vector3d& point) const {
    return {
        static_cast<int32_t>(std::floor(point.x() / voxel_size_)),
        static_cast<int32_t>(std::floor(point.y() / voxel_size_)),
        static_cast<int32_t>(std::floor(point.z() / voxel_size_))
    };
}

void VoxelHashMap::addPoints(const std::vector<Eigen::Vector3d>& points) {
    for (const auto& p : points) {
        auto key = toKey(p);
        auto& voxel = map_[key];
        if (voxel.size() < max_points_per_voxel_) {
            voxel.push_back(p);
        }
    }
}

void VoxelHashMap::clear() {
    map_.clear();
}

std::vector<Eigen::Vector3d> VoxelHashMap::getPointsNear(
    const Eigen::Vector3d& query, double radius) const {

    std::vector<Eigen::Vector3d> result;
    int search_range = static_cast<int>(std::ceil(radius / voxel_size_));
    auto center_key = toKey(query);
    double radius_sq = radius * radius;

    for (int dx = -search_range; dx <= search_range; ++dx) {
        for (int dy = -search_range; dy <= search_range; ++dy) {
            for (int dz = -search_range; dz <= search_range; ++dz) {
                VoxelKey key{center_key.x + dx, center_key.y + dy, center_key.z + dz};
                auto it = map_.find(key);
                if (it != map_.end()) {
                    for (const auto& p : it->second) {
                        if ((p - query).squaredNorm() <= radius_sq) {
                            result.push_back(p);
                        }
                    }
                }
            }
        }
    }
    return result;
}

std::vector<Eigen::Vector3d> VoxelHashMap::getAllPoints() const {
    std::vector<Eigen::Vector3d> result;
    for (const auto& [key, points] : map_) {
        result.insert(result.end(), points.begin(), points.end());
    }
    return result;
}

size_t VoxelHashMap::size() const {
    size_t total = 0;
    for (const auto& [key, points] : map_) {
        total += points.size();
    }
    return total;
}

void VoxelHashMap::removePointsFarFrom(const Eigen::Vector3d& center, double max_distance) {
    double max_dist_sq = max_distance * max_distance;
    for (auto it = map_.begin(); it != map_.end(); ) {
        Eigen::Vector3d voxel_center(
            (it->first.x + 0.5) * voxel_size_,
            (it->first.y + 0.5) * voxel_size_,
            (it->first.z + 0.5) * voxel_size_);

        if ((voxel_center - center).squaredNorm() > max_dist_sq) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace g1_slam
