#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <cstdint>

namespace g1_slam {

struct VoxelKey {
    int32_t x, y, z;

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        // Spatial hash combining all three axes
        size_t h = 73856093ULL * static_cast<size_t>(k.x) ^
                   19349669ULL * static_cast<size_t>(k.y) ^
                   83492791ULL * static_cast<size_t>(k.z);
        return h;
    }
};

class VoxelHashMap {
public:
    explicit VoxelHashMap(double voxel_size = 0.5);

    void addPoints(const std::vector<Eigen::Vector3d>& points);
    void clear();

    std::vector<Eigen::Vector3d> getPointsNear(const Eigen::Vector3d& query, double radius) const;
    std::vector<Eigen::Vector3d> getAllPoints() const;
    size_t size() const;

    void removePointsFarFrom(const Eigen::Vector3d& center, double max_distance);

    double voxelSize() const { return voxel_size_; }

private:
    VoxelKey toKey(const Eigen::Vector3d& point) const;

    double voxel_size_;
    size_t max_points_per_voxel_ = 20;
    std::unordered_map<VoxelKey, std::vector<Eigen::Vector3d>, VoxelKeyHash> map_;
};

}  // namespace g1_slam
